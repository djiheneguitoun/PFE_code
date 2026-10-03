/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
// Scénario ns-3 « Wi-Fi entre drones » : N drones en Wi-Fi 802.11n ad hoc ; toutes les 0,5 s, relit
// leurs positions dans un CSV et écrit pour chaque paire la distance, le RSSI (puissance reçue) et la latence.
// Tourne en temps réel pour suivre un vol en direct. Lancé par scripts/08_wifi_bridge.py et scripts/12_ns3_bridge.py.
// À copier dans scratch/ de ns-3.40, puis depuis ~/ns-allinone-3.40/ns-3.40 :
//   ./ns3 build && ./ns3 run "drone-wifi-scenario --nDrones=3 --simTime=60"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/propagation-module.h"


// Active le code du module NS3-Sionna (contrib/sionna, installé par scripts/installation/05)
#define NS3_SIONNA_AVAILABLE
#ifdef NS3_SIONNA_AVAILABLE
#include "ns3/sionna-helper.h"
#include "ns3/sionna-propagation-cache.h"
#include "ns3/sionna-propagation-delay-model.h"
#include "ns3/sionna-propagation-loss-model.h"
#include "ns3/sionna-mobility-model.h"
#include "ns3/sionna-utils.h"
#endif

#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("DroneWifiScenario");


// Valeurs par défaut des options de la ligne de commande (voir main)
// Nombre de drones
static uint32_t g_nDrones = 3;
// CSV d'entrée : une ligne d'en-tête, puis id,x,y,z (m) ; réécrit en continu par le script de vol
static std::string g_posFile = "/tmp/drone_positions.csv";
// CSV de sortie : une ligne par paire de drones et par mise à jour
static std::string g_outFile = "/tmp/ns3_output.csv";
// Durée de la simulation, en s (temps réel : autant de secondes d'horloge)
static double g_simTime = 60.0;
// Période de relecture des positions, en s
static double g_updateInterval = 0.5;
// Modèle de propagation : "log-distance" (formule simple) ou "sionna" (lancer de rayons)
static std::string g_channelModel = "log-distance";
// Pour "sionna" : scène 3D (chemin relatif côté serveur Sionna), puis adresse ZMQ du serveur Python
static std::string g_sionnaEnv = "simple_room/simple_room.xml"; 
static std::string g_sionnaUrl = "tcp://localhost:5555";      

// Nœuds ns-3 (un par drone), leurs cartes Wi-Fi et le fichier de sortie ouvert
static NodeContainer g_droneNodes;
static NetDeviceContainer g_wifiDevices;
static std::ofstream g_outputCsv;

//  Clés pour le RSSI et la latence 
static Ptr<PropagationLossModel> g_propLossModel;    
static double g_txPowerDbm = 20.0;                  
static Ptr<FlowMonitor> g_flowMonitor;              
static Ptr<Ipv4FlowClassifier> g_flowClassifier;    

// Puissance d'émission en dBm (g_txPowerDbm), modèle de propagation choisi, et FlowMonitor
// (compteur de paquets et de délais par flux) qui fournit la latence

// Stats FlowMonitor précédentes
static FlowMonitor::FlowStatsContainer g_prevFlowStats;

// Pointeur global vers SionnaHelper 
#ifdef NS3_SIONNA_AVAILABLE
static SionnaHelper* g_sionnaHelper = nullptr;
#endif

// Position d'un drone, en m
struct DronePos {
    double x, y, z;
};

// Lit le CSV des positions (en-tête puis id,x,y,z) et renvoie un vecteur de nDrones positions ;
// un drone absent du fichier (ou fichier illisible) reste en (0, 0, 0).
static std::vector<DronePos>
ReadPositions(const std::string& filename, uint32_t nDrones)
{
    std::vector<DronePos> positions(nDrones, {0.0, 0.0, 0.0});
    std::ifstream file(filename);
    if (!file.is_open()) {
        NS_LOG_WARN("Cannot open position file: " << filename);
        return positions;
    }

    std::string line;
    // Saute la 1re ligne (l'en-tête)
    if (std::getline(file, line)) {
    }

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        uint32_t id;
        double x, y, z;

        std::getline(ss, token, ','); id = std::stoi(token);
        std::getline(ss, token, ','); x = std::stod(token);
        std::getline(ss, token, ','); y = std::stod(token);
        std::getline(ss, token, ','); z = std::stod(token);

        if (id < nDrones) {
            positions[id] = {x, y, z};
        }
    }
    return positions;
}

// Renvoie la latence en ms de chaque paire de drones (min, max), d'après FlowMonitor : délai moyen
// des seuls paquets reçus depuis l'appel précédent (pas la moyenne depuis le début) ; garde le plus petit des deux sens.
static std::map<std::pair<uint32_t, uint32_t>, double>
GetPairLatencies(void)
{
    std::map<std::pair<uint32_t, uint32_t>, double> result;

    if (!g_flowMonitor || !g_flowClassifier) {
        return result;
    }

    g_flowMonitor->CheckForLostPackets();
    FlowMonitor::FlowStatsContainer stats = g_flowMonitor->GetFlowStats();

    for (auto& iter : stats) {
        Ipv4FlowClassifier::FiveTuple t = g_flowClassifier->FindFlow(iter.first);

        // Mapper IP → index drone : 10.1.1.(i+1) → drone i
        uint32_t srcByte = t.sourceAddress.Get() & 0xFF;
        uint32_t dstByte = t.destinationAddress.Get() & 0xFF;
        if (srcByte == 0 || dstByte == 0) continue;
        uint32_t srcDrone = srcByte - 1;
        uint32_t dstDrone = dstByte - 1;
        if (srcDrone >= g_nDrones || dstDrone >= g_nDrones) continue;

        // Calculer le délai récent (delta depuis la dernière mesure)
        double delayMs = 0.0;
        auto prevIt = g_prevFlowStats.find(iter.first);
        if (prevIt != g_prevFlowStats.end()) {
            uint64_t newPkts = iter.second.rxPackets - prevIt->second.rxPackets;
            if (newPkts > 0) {
                Time newDelay = iter.second.delaySum - prevIt->second.delaySum;
                delayMs = newDelay.GetSeconds() * 1000.0 / (double)newPkts;
            }
        } else if (iter.second.rxPackets > 0) {
            delayMs = iter.second.delaySum.GetSeconds() * 1000.0
                      / (double)iter.second.rxPackets;
        }

        // Stocker comme paire ordonnée (min, max)
        auto key = std::make_pair(std::min(srcDrone, dstDrone),
                                   std::max(srcDrone, dstDrone));
        if (delayMs > 0.0) {
            auto existing = result.find(key);
            if (existing == result.end() || delayMs < existing->second) {
                result[key] = delayMs;
            }
        }
    }

    // Sauvegarder pour le prochain delta
    g_prevFlowStats = stats;

    return result;
}

// Relit les positions, déplace les nœuds, écrit distance, RSSI et latence de chaque paire dans le CSV,
// puis se replanifie g_updateInterval secondes plus tard.
static void
UpdatePositions(void)
{
    // 1. Lire les positions depuis le CSV 
    std::vector<DronePos> positions = ReadPositions(g_posFile, g_nDrones);

    // 2. Mettre à jour les modèles de mobilité NS-3
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        Ptr<MobilityModel> mob = g_droneNodes.Get(i)->GetObject<MobilityModel>();
        if (mob) {
            mob->SetPosition(Vector(positions[i].x, positions[i].y, positions[i].z));
        }
    }

    double simTime = Simulator::Now().GetSeconds();

    // 3. Latence réelle depuis le FlowMonitor
    auto pairLatencies = GetPairLatencies();

    // 4. Métriques pour chaque paire de drones
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        for (uint32_t j = i + 1; j < g_nDrones; ++j) {
            // --- Distance ---
            double dx = positions[i].x - positions[j].x;
            double dy = positions[i].y - positions[j].y;
            double dz = positions[i].z - positions[j].z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

          
            // --- RSSI : puissance reçue (dBm) calculée par le modèle de propagation, sans paquet ---
            Ptr<MobilityModel> mob_i = g_droneNodes.Get(i)->GetObject<MobilityModel>();
            Ptr<MobilityModel> mob_j = g_droneNodes.Get(j)->GetObject<MobilityModel>();
            double rssiDbm = g_propLossModel->CalcRxPower(g_txPowerDbm, mob_i, mob_j);

            // --- Latence depuis le FlowMonitor (mesurée) ---
            double latencyMs = 0.0;
            auto pairKey = std::make_pair(i, j);
            auto latIt = pairLatencies.find(pairKey);
            if (latIt != pairLatencies.end() && latIt->second > 0.0) {
                latencyMs = latIt->second;
            } else {
                // aucun paquet mesuré pour cette paire : estimation = temps de vol de l'onde + 2 ms
                double propagationMs = (distance / 3e8) * 1000.0;
                latencyMs = propagationMs + 2.0;
            }

            // --- Écriture CSV ---
            g_outputCsv << simTime << ","
                       << i << "," << j << ","
                       << distance << ","
                       << rssiDbm << ","
                       << latencyMs << ","
                       << positions[i].x << "," << positions[i].y << "," << positions[i].z << ","
                       << positions[j].x << "," << positions[j].y << "," << positions[j].z
                       << std::endl;

            NS_LOG_INFO("t=" << simTime
                       << " D" << i << "<->D" << j
                       << " dist=" << distance << "m"
                       << " RSSI=" << rssiDbm << "dBm"
                       << " lat=" << latencyMs << "ms");
        }
    }

    // Prochain update
    Simulator::Schedule(Seconds(g_updateInterval), &UpdatePositions);
}

// Lit les options, construit le réseau Wi-Fi des drones, lance la simulation en temps réel
// pendant g_simTime secondes et écrit le CSV de sortie.
int
main(int argc, char *argv[])
{
    CommandLine cmd;
    cmd.AddValue("nDrones", "Number of drones", g_nDrones);
    cmd.AddValue("posFile", "Input CSV with drone positions", g_posFile);
    cmd.AddValue("outFile", "Output CSV with RSSI and latency", g_outFile);
    cmd.AddValue("simTime", "Simulation time in seconds", g_simTime);
    cmd.AddValue("updateInterval", "Position update interval (s)", g_updateInterval);
    cmd.AddValue("channelModel", "Channel model: log-distance or sionna", g_channelModel);
    cmd.AddValue("sionnaEnv", "Sionna XML scene file (relative path)", g_sionnaEnv);
    cmd.AddValue("sionnaUrl", "Sionna ZMQ server URL", g_sionnaUrl);
    cmd.Parse(argc, argv);

    // Temps réel : 1 s simulée dure 1 s d'horloge, pour suivre le vol en direct
    GlobalValue::Bind("SimulatorImplementationType",
                      StringValue("ns3::RealtimeSimulatorImpl"));

    NS_LOG_INFO("=== Drone WiFi Scenario (NS-3 + NS3-Sionna) ===");
    NS_LOG_INFO("Drones: " << g_nDrones);
    NS_LOG_INFO("Channel model: " << g_channelModel);
    NS_LOG_INFO("Position file: " << g_posFile);
    NS_LOG_INFO("Output file: " << g_outFile);
    NS_LOG_INFO("Sim time: " << g_simTime << "s (real-time)");

    // Créer les nœuds 
    g_droneNodes.Create(g_nDrones);

    //  WiFi 
    // Wi-Fi 802.11n à débit fixe : MCS 7 pour les données, MCS 0 pour les trames de contrôle
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                  "DataMode", StringValue("HtMcs7"),
                                  "ControlMode", StringValue("HtMcs0"));

    // PHY
    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("TxPowerStart", DoubleValue(g_txPowerDbm));
    wifiPhy.Set("TxPowerEnd", DoubleValue(g_txPowerDbm));

    
    // Canal radio partagé par tous les drones
    Ptr<YansWifiChannel> wifiChannel = CreateObject<YansWifiChannel>();

    // Modèle de délai de propagation
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    wifiChannel->SetPropagationDelayModel(delayModel);

    // Modèle de perte de propagation 
    if (g_channelModel == "sionna") {
#ifdef NS3_SIONNA_AVAILABLE
        
        NS_LOG_INFO("*** Utilisation de NS3-Sionna (ray-tracing) ***");
        NS_LOG_INFO("Scène Sionna : " << g_sionnaEnv);
        NS_LOG_INFO("Serveur ZMQ  : " << g_sionnaUrl);

        // Client du serveur Python Sionna (à lancer à part, cf. scripts/installation/05) ;
        // mode P2P : propagation calculée pour chaque paire émetteur-récepteur
        g_sionnaHelper = new SionnaHelper(g_sionnaEnv, g_sionnaUrl);

       
        g_sionnaHelper->SetMode(SionnaHelper::MODE_P2P);

        Ptr<SionnaPropagationCache> propagationCache =
            CreateObject<SionnaPropagationCache>();
        propagationCache->SetSionnaHelper(*g_sionnaHelper);
        propagationCache->SetCaching(true);

        Ptr<SionnaPropagationDelayModel> sionnaDelay =
            CreateObject<SionnaPropagationDelayModel>();
        sionnaDelay->SetPropagationCache(propagationCache);
        wifiChannel->SetPropagationDelayModel(sionnaDelay);

        Ptr<SionnaPropagationLossModel> sionnaLoss =
            CreateObject<SionnaPropagationLossModel>();
        sionnaLoss->SetPropagationCache(propagationCache);
        g_propLossModel = sionnaLoss;
#else
        NS_LOG_WARN("NS3-Sionna non compilé ! Fallback vers LogDistance.");
        NS_LOG_WARN("Pour activer : #define NS3_SIONNA_AVAILABLE dans drone-wifi-scenario.cc");
        NS_LOG_WARN("Et compiler NS3-Sionna : cd ns-3.40 && ./ns3 build");
        Ptr<LogDistancePropagationLossModel> lossModel =
            CreateObject<LogDistancePropagationLossModel>();
        lossModel->SetAttribute("Exponent", DoubleValue(3.0));
        lossModel->SetAttribute("ReferenceLoss", DoubleValue(40.0));
        g_propLossModel = lossModel;
#endif
    } else {
        NS_LOG_INFO("Modèle Log-Distance (indoor warehouse)");
        // Perte en log-distance : exposant 3 (intérieur encombré), 40 dB à la distance de référence (1 m par défaut)
        Ptr<LogDistancePropagationLossModel> lossModel =
            CreateObject<LogDistancePropagationLossModel>();
        lossModel->SetAttribute("Exponent", DoubleValue(3.0));
        lossModel->SetAttribute("ReferenceLoss", DoubleValue(40.0));
        g_propLossModel = lossModel;
    }

    wifiChannel->SetPropagationLossModel(g_propLossModel);
    wifiPhy.SetChannel(wifiChannel);

    // Mode ad hoc : les drones se parlent directement, sans point d'accès
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    g_wifiDevices = wifi.Install(wifiPhy, wifiMac, g_droneNodes);

    InternetStackHelper internet;
    internet.Install(g_droneNodes);

    // Adresses IP 10.1.1.1, 10.1.1.2… : le drone i reçoit 10.1.1.(i+1)
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(g_wifiDevices);

    // positions lues depuis le CSV
    MobilityHelper mobility;
#ifdef NS3_SIONNA_AVAILABLE
    if (g_channelModel == "sionna") {
        // On utilise le mode "constant position" de Sionna
        mobility.SetMobilityModel("ns3::SionnaMobilityModel");
    } else {
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }
#else
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
#endif
    mobility.Install(g_droneNodes);

    // Positions initiales
    std::vector<DronePos> initPos = ReadPositions(g_posFile, g_nDrones);
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        Ptr<MobilityModel> mob = g_droneNodes.Get(i)->GetObject<MobilityModel>();
        mob->SetPosition(Vector(initPos[i].x, initPos[i].y, initPos[i].z));
    }

#ifdef NS3_SIONNA_AVAILABLE
   
    // Donne à Sionna les paramètres radio du Wi-Fi (fréquence, largeur de canal, FFT, sous-porteuses) et le démarre
    if (g_channelModel == "sionna" && g_sionnaHelper != nullptr) {
        double channelWidth = get_channel_width(g_wifiDevices.Get(0));
        int centerFreq = (int)get_center_freq(g_wifiDevices.Get(0));
        int fftSize = getFFTSize(WIFI_STANDARD_80211n, channelWidth);
        int subcarrierSpacing = getSubcarrierSpacing(WIFI_STANDARD_80211n);

        NS_LOG_INFO("Sionna Configure: freq=" << centerFreq
                    << "MHz, bw=" << channelWidth
                    << "MHz, fft=" << fftSize
                    << ", scs=" << subcarrierSpacing << "Hz");

        g_sionnaHelper->Configure(centerFreq, (int)channelWidth,
                                   fftSize, subcarrierSpacing, 500);
        g_sionnaHelper->Start();
        NS_LOG_INFO("Sionna démarré et connecté au serveur !");
    }
#endif

    //  Trafic UDP Echo entre drones 
    // Chaque drone i envoie un paquet UDP de 256 octets toutes les 0,1 s (à partir de t = 1 s) au drone
    // suivant j = i+1 (en anneau) ; FlowMonitor mesure le délai de chaque flux i -> j.
    uint16_t port = 9;
    for (uint32_t i = 0; i < g_nDrones; ++i) {
        uint32_t j = (i + 1) % g_nDrones;

        Ptr<Node> srcNode = g_droneNodes.Get(i);
        Ptr<Node> dstNode = g_droneNodes.Get(j);

        Ptr<Ipv4> dstIpv4 = dstNode->GetObject<Ipv4>();
        Ipv4Address dstAddr = dstIpv4->GetAddress(1, 0).GetLocal();

        // Serveur UDP Echo sur le drone destination
        UdpEchoServerHelper echoServer(port + i);
        ApplicationContainer serverApp = echoServer.Install(dstNode);
        serverApp.Start(Seconds(0.0));
        serverApp.Stop(Seconds(g_simTime));

       
        // Client UDP Echo sur le drone source
        UdpEchoClientHelper echoClient(dstAddr, port + j);
        echoClient.SetAttribute("MaxPackets", UintegerValue(999999));
        echoClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
        echoClient.SetAttribute("PacketSize", UintegerValue(256));

        ApplicationContainer clientApp = echoClient.Install(srcNode);
        clientApp.Start(Seconds(1.0));
        clientApp.Stop(Seconds(g_simTime));
    }

    //  Ouvrir le CSV de sortie 
    g_outputCsv.open(g_outFile);
    g_outputCsv << "time_s,drone_i,drone_j,distance_m,rssi_dbm,latency_ms,"
                << "xi,yi,zi,xj,yj,zj" << std::endl;

    //  FlowMonitor pour la  latence 
    FlowMonitorHelper flowHelper;
    g_flowMonitor = flowHelper.InstallAll();
    g_flowClassifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());

    //  Planifier les mises à jour de position 
    Simulator::Schedule(Seconds(0.0), &UpdatePositions);

    //  Lancer la simulation 
    Simulator::Stop(Seconds(g_simTime));
    Simulator::Run();

    //  Statistiques finales des flux 
    // (comme tous les NS_LOG_INFO, visibles seulement avec NS_LOG=DroneWifiScenario=info)
    NS_LOG_INFO("\n=== Statistiques FlowMonitor ===");
    g_flowMonitor->CheckForLostPackets();
    FlowMonitor::FlowStatsContainer stats = g_flowMonitor->GetFlowStats();

    for (auto &iter : stats) {
        Ipv4FlowClassifier::FiveTuple t = g_flowClassifier->FindFlow(iter.first);
        double meanDelayMs = 0;
        if (iter.second.rxPackets > 0) {
            meanDelayMs = iter.second.delaySum.GetSeconds() * 1000.0
                          / (double)iter.second.rxPackets;
        }
        NS_LOG_INFO("Flow " << iter.first
                    << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")"
                    << " Tx=" << iter.second.txPackets
                    << " Rx=" << iter.second.rxPackets
                    << " Lost=" << iter.second.lostPackets
                    << " MeanDelay=" << meanDelayMs << "ms");
    }

    g_outputCsv.close();

#ifdef NS3_SIONNA_AVAILABLE
    if (g_sionnaHelper) {
        g_sionnaHelper->Destroy();
        delete g_sionnaHelper;
        g_sionnaHelper = nullptr;
    }
#endif

    Simulator::Destroy();

    NS_LOG_INFO("Output saved to: " << g_outFile);
    return 0;
}
