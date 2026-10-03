// Logique de la page du tableau de bord « WiFi / 5G + exploration » (chargée par index.html).
// Toutes les 2 s, demande /api/data au serveur dashboard_server.py puis redessine les cartes
// (canvas), les graphiques Chart.js, les tableaux et le journal de la page.
// Rien à lancer à part le serveur : python3 dashboard/dashboard_server.py, puis http://localhost:8050

// Adresse de l'API : le même serveur que celui qui a envoyé la page.
const API = window.location.origin + '/api';
// Période d'interrogation du serveur, en ms (2 s).
const POLL_MS = 2000;
// Nombre de ticks gardés sur les courbes (tick = cycle de mesure des scripts 08/09, ou step de l'AIF).
const MAX_HISTORY = 120;       // keep last 120 ticks on charts

// Ajoute roundRect (rectangle à coins arrondis) aux anciens navigateurs qui ne l'ont pas.
if (!CanvasRenderingContext2D.prototype.roundRect) {
    // Trace le contour d'un rectangle (x, y, largeur w, hauteur h, en px) à coins de rayon r.
    CanvasRenderingContext2D.prototype.roundRect = function (x, y, w, h, r) {
        if (w < 2 * r) r = w / 2;
        if (h < 2 * r) r = h / 2;
        this.moveTo(x + r, y);
        this.arcTo(x + w, y, x + w, y + h, r);
        this.arcTo(x + w, y + h, x, y + h, r);
        this.arcTo(x, y + h, x, y, r);
        this.arcTo(x, y, x + w, y, r);
        this.closePath();
    };
}

// ─── Couleurs communes (WiFi en cyan, 5G en violet) ───
const C = {
    wifi: 'rgba(34,211,238,1)',
    wifiBg: 'rgba(34,211,238,.15)',
    fiveg: 'rgba(167,139,250,1)',
    fivegBg: 'rgba(167,139,250,.15)',
    green: 'rgba(52,211,153,1)',
    greenBg: 'rgba(52,211,153,.15)',
    red: 'rgba(248,113,113,1)',
    orange: 'rgba(251,146,60,1)',
    orangeBg: 'rgba(251,146,60,.12)',
    yellow: 'rgba(251,191,36,1)',
    grid: 'rgba(255,255,255,.06)',
    text: 'rgba(160,174,192,.7)',
};

// ─── Réglages communs à tous les graphiques Chart.js ───
Chart.defaults.color = C.text;
Chart.defaults.borderColor = C.grid;
Chart.defaults.font.family = "'Inter', sans-serif";
Chart.defaults.font.size = 11;
Chart.defaults.elements.point.radius = 2;
Chart.defaults.elements.point.hoverRadius = 5;
Chart.defaults.animation.duration = 400;

// Couleurs par paire de drones, une pour le WiFi et une pour la 5G (6 paires, puis on recommence)
const PAIR_COLORS = [
    { wifi: '#22d3ee', fiveg: '#a78bfa' },
    { wifi: '#34d399', fiveg: '#f472b6' },
    { wifi: '#fbbf24', fiveg: '#fb923c' },
    { wifi: '#60a5fa', fiveg: '#e879f9' },
    { wifi: '#a3e635', fiveg: '#f87171' },
    { wifi: '#2dd4bf', fiveg: '#c084fc' },
];

// ─── État de la page ───
// Onglet affiché : 'explore' (Exploration), 'both' (Comparaison), 'wifi' ou '5g'.
let activeScenario = 'explore';   // 'explore' | 'both' | 'wifi' | '5g'

// Dernières données reçues et historique des courbes (une liste de valeurs par paire ou par drone).
let state = {
    tick: 0,
    history: {
        labels: [],
        rssiWifi: {},     // { "0↔1": [values] }
        rssi5g: {},       // { "D0": [values] }
        latWifi: {},
        lat5g: {},
    },
    currentWifi: null,
    current5g: null,
    connected: false,
};

// Historique de l'exploration (pourcentages, entropies, numéros de step).
let explState = {
    historyPct: [],
    historyEntropy: [],
    historyLabels: [],
    prevExplStep: -1,
};

// ─── Filtre par onglet ───
// Montre seulement les blocs dont l'attribut data-vis contient l'onglet choisi, puis redessine tout.
function applyFilter(scenario) {
    activeScenario = scenario;
    // Montre / cache chaque élément selon son attribut data-vis
    document.querySelectorAll('[data-vis]').forEach(el => {
        const vis = el.getAttribute('data-vis').split(/\s+/);
        if (vis.includes(scenario)) {
            el.style.display = '';
            el.classList.remove('hidden-by-filter');
        } else {
            el.style.display = 'none';
            el.classList.add('hidden-by-filter');
        }
    });
    // Redessine les graphiques avec les seules séries de l'onglet
    updateTimeCharts();
    if (state.currentWifi || state.current5g) {
        updateComparisonCharts(state.currentWifi, state.current5g);
        drawMap(state.currentWifi, state.current5g, state.livePositions || {});
    }
    // Recalcule la taille des canvas 50 ms plus tard (la mise en page a pu changer)
    setTimeout(() => {
        resizeExploreCanvases();
        resizeMap();
    }, 50);
}

// ─────────────────────────────────────────────────────────────
// GRAPHIQUES (onglets Comparaison, WiFi et 5G)
// ─────────────────────────────────────────────────────────────

// Crée un graphique en courbes vide sur le canvas canvasId (titre d'axe yLabel, unité ajoutée aux infobulles).
function makeTimeChart(canvasId, yLabel, unit = '') {
    const ctx = document.getElementById(canvasId).getContext('2d');
    return new Chart(ctx, {
        type: 'line',
        data: { labels: [], datasets: [] },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            interaction: { mode: 'index', intersect: false },
            plugins: {
                legend: { display: true, position: 'top', labels: { usePointStyle: true, pointStyle: 'circle', boxWidth: 6, padding: 14, font: { size: 11 } } },
                tooltip: {
                    backgroundColor: 'rgba(17,22,32,.92)',
                    borderColor: 'rgba(255,255,255,.1)', borderWidth: 1,
                    titleFont: { weight: 600 },
                    // Texte de l'infobulle : « nom de la série : valeur à 2 décimales + unité ».
                    callbacks: { label: (c) => `${c.dataset.label}: ${c.parsed.y?.toFixed(2)}${unit}` }
                }
            },
            scales: {
                x: { grid: { display: false }, ticks: { maxTicksLimit: 12, maxRotation: 0 } },
                y: { grid: { color: C.grid }, title: { display: true, text: yLabel, font: { size: 11 } } }
            }
        }
    });
}

// Crée un graphique en barres vide sur le canvas canvasId (titre d'axe yLabel).
function makeBarChart(canvasId, yLabel) {
    const ctx = document.getElementById(canvasId).getContext('2d');
    return new Chart(ctx, {
        type: 'bar',
        data: { labels: [], datasets: [] },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: { display: true, position: 'top', labels: { usePointStyle: true, pointStyle: 'rect', boxWidth: 10, padding: 14 } },
                tooltip: { backgroundColor: 'rgba(17,22,32,.92)', borderColor: 'rgba(255,255,255,.1)', borderWidth: 1 }
            },
            scales: {
                x: { grid: { display: false } },
                y: { grid: { color: C.grid }, title: { display: true, text: yLabel, font: { size: 11 } } }
            }
        }
    });
}

// Les cinq graphiques réseau : 2 courbes dans le temps (RSSI = puissance reçue, latence), 3 barres de comparaison.
const chartRSSI = makeTimeChart('chartRSSI', 'RSSI (dBm)', ' dBm');
const chartLatency = makeTimeChart('chartLatency', 'Latence (ms)', ' ms');
const chartCompRSSI = makeBarChart('chartCompareRSSI', 'RSSI (dBm)');
const chartCompLat = makeBarChart('chartCompareLatency', 'Latence (ms)');
const chartDist = makeBarChart('chartDistance', 'Distance (m)');

// ─── Canvas de l'onglet Exploration : carte globale, une carte par drone (3), quatre jauges ───
const heroCanvas = document.getElementById('canvasHeroMap');
const heroCtx = heroCanvas ? heroCanvas.getContext('2d') : null;
const droneCanvases = [0, 1, 2].map(i => document.getElementById(`canvasDrone${i}`));
const droneCtxs = droneCanvases.map(c => c ? c.getContext('2d') : null);
const gaugeProgressCanvas = document.getElementById('canvasGaugeProgress');
const gaugeProgressCtx = gaugeProgressCanvas ? gaugeProgressCanvas.getContext('2d') : null;
const gaugeAltCanvas = document.getElementById('canvasGaugeAlt');
const gaugeAltCtx = gaugeAltCanvas ? gaugeAltCanvas.getContext('2d') : null;
const gaugeSpeedCanvas = document.getElementById('canvasGaugeSpeed');
const gaugeSpeedCtx = gaugeSpeedCanvas ? gaugeSpeedCanvas.getContext('2d') : null;
const gaugeDistCanvas = document.getElementById('canvasGaugeDist');
const gaugeDistCtx = gaugeDistCanvas ? gaugeDistCanvas.getContext('2d') : null;

// Donne à chaque canvas d'exploration la taille de son conteneur (hauteur minimale 180 px).
function resizeExploreCanvases() {
    const all = [heroCanvas, ...droneCanvases, gaugeProgressCanvas, gaugeAltCanvas, gaugeSpeedCanvas, gaugeDistCanvas];
    all.forEach(c => {
        if (!c || !c.parentElement) return;
        const rect = c.parentElement.getBoundingClientRect();
        c.width = Math.floor(rect.width);
        c.height = Math.max(Math.floor(rect.height), 180);
    });
}
resizeExploreCanvases();
window.addEventListener('resize', resizeExploreCanvases);

// ─────────────────────────────────────────────────────────────
// CARTE 2D DES POSITIONS (onglets réseau)
// ─────────────────────────────────────────────────────────────

const mapCanvas = document.getElementById('canvasMap');
const mapCtx = mapCanvas.getContext('2d');

// Ajuste la carte 2D à son conteneur (marge de 16 px, hauteur minimale 260 px).
function resizeMap() {
    const rect = mapCanvas.parentElement.getBoundingClientRect();
    mapCanvas.width = rect.width - 16;
    mapCanvas.height = Math.max(rect.height - 16, 260);
}
resizeMap();
window.addEventListener('resize', resizeMap);

// Couleur de chaque drone sur la carte 2D ; position par défaut (m) de l'antenne 5G, comme dans dashboard_server.py.
const DRONE_COLORS = ['#22d3ee', '#34d399', '#a78bfa', '#fbbf24', '#f472b6', '#fb923c'];
const GNB_POS = { x: 0, y: 0, z: 6 };

// Dessine la carte 2D : grille, contour de l'entrepôt, antenne 5G, liens WiFi et 5G, drones avec leur altitude.
function drawMap(wifiData, fivegData, data_positions) {
    const W = mapCanvas.width, H = mapCanvas.height;
    mapCtx.clearRect(0, 0, W, H);

    // Limites approximatives de l'entrepôt : x ∈ [-8, 8] m, y ∈ [-5, 5] m (zone affichée 20 m × 14 m)
    const scale = Math.min(W / 20, H / 14);
    const cx = W / 2, cy = H / 2;
    // Convertit une abscisse x (m) en pixel horizontal (origine au centre du canvas).
    const toX = (x) => cx + x * scale;
    // Convertit une ordonnée y (m) en pixel vertical (axe inversé : y vers le haut).
    const toY = (y) => cy - y * scale;  // invert Y

    // Grille : un trait tous les 2 m
    mapCtx.strokeStyle = 'rgba(255,255,255,.04)';
    mapCtx.lineWidth = 1;
    for (let gx = -8; gx <= 8; gx += 2) {
        mapCtx.beginPath(); mapCtx.moveTo(toX(gx), 0); mapCtx.lineTo(toX(gx), H); mapCtx.stroke();
    }
    for (let gy = -5; gy <= 5; gy += 2) {
        mapCtx.beginPath(); mapCtx.moveTo(0, toY(gy)); mapCtx.lineTo(W, toY(gy)); mapCtx.stroke();
    }

    // Contour de l'entrepôt : rectangle de 14 m × 9 m centré sur l'origine
    mapCtx.strokeStyle = 'rgba(255,255,255,.1)';
    mapCtx.lineWidth = 2;
    mapCtx.strokeRect(toX(-7), toY(4.5), 14 * scale, 9 * scale);

    // Positions des drones : fichier de positions d'abord, puis remplacées par les mesures WiFi et 5G
    let positions = {};

    // 1. Positions lues dans /tmp/drone_positions.csv
    if (data_positions) {
        Object.entries(data_positions).forEach(([id, pos]) => {
            positions[parseInt(id)] = pos;
        });
    }

    // 2. Remplacées / complétées par celles des paires WiFi
    if (wifiData && wifiData.pairs) {
        wifiData.pairs.forEach(p => {
            if (p.pos_a) positions[p.drone_a] = p.pos_a;
            if (p.pos_b) positions[p.drone_b] = p.pos_b;
        });
    }
    // 3. Puis par celles des mesures 5G
    if (fivegData && fivegData.drones) {
        fivegData.drones.forEach(d => {
            positions[d.id] = { x: d.x, y: d.y, z: d.z };
        });
    }

    // Antenne 5G (gNB)
    if (fivegData) {
        const gnb = fivegData.gnb || GNB_POS;
        const gx = toX(gnb.x), gy = toY(gnb.y);
        mapCtx.fillStyle = 'rgba(167,139,250,.3)';
        mapCtx.beginPath(); mapCtx.arc(gx, gy, 18, 0, Math.PI * 2); mapCtx.fill();
        mapCtx.fillStyle = '#a78bfa';
        mapCtx.beginPath(); mapCtx.arc(gx, gy, 6, 0, Math.PI * 2); mapCtx.fill();
        mapCtx.fillStyle = '#a78bfa';
        mapCtx.font = '600 11px Inter';
        mapCtx.textAlign = 'center';
        mapCtx.fillText('gNB', gx, gy - 14);
    }

    // Liens WiFi entre drones (pointillés rouges si le signal est bloqué)
    if (wifiData && wifiData.pairs) {
        wifiData.pairs.forEach((p, i) => {
            const a = positions[p.drone_a], b = positions[p.drone_b];
            if (!a || !b) return;
            const blocked = p.rssi === 'blocked' || p.rssi === 'error';
            mapCtx.strokeStyle = blocked ? 'rgba(248,113,113,.3)' : 'rgba(34,211,238,.2)';
            mapCtx.lineWidth = blocked ? 1 : 2;
            mapCtx.setLineDash(blocked ? [4, 4] : []);
            mapCtx.beginPath();
            mapCtx.moveTo(toX(a.x), toY(a.y));
            mapCtx.lineTo(toX(b.x), toY(b.y));
            mapCtx.stroke();
            mapCtx.setLineDash([]);
        });
    }

    // Liens 5G drone → antenne (rouges si bloqués)
    if (fivegData && fivegData.drones) {
        const gnb = fivegData.gnb || GNB_POS;
        fivegData.drones.forEach(d => {
            const blocked = d.rssi === null || d.rssi === 'BLOCKED';
            mapCtx.strokeStyle = blocked ? 'rgba(248,113,113,.2)' : 'rgba(167,139,250,.15)';
            mapCtx.lineWidth = 1;
            mapCtx.setLineDash([3, 3]);
            mapCtx.beginPath();
            mapCtx.moveTo(toX(d.x), toY(d.y));
            mapCtx.lineTo(toX(gnb.x), toY(gnb.y));
            mapCtx.stroke();
            mapCtx.setLineDash([]);
        });
    }

    // Drones : halo, point, nom et altitude
    const droneIds = Object.keys(positions).map(Number).sort();
    droneIds.forEach((did, idx) => {
        const p = positions[did];
        const px = toX(p.x), py = toY(p.y);
        const col = DRONE_COLORS[idx % DRONE_COLORS.length];

        // Halo
        const grad = mapCtx.createRadialGradient(px, py, 0, px, py, 22);
        grad.addColorStop(0, col + '33');
        grad.addColorStop(1, col + '00');
        mapCtx.fillStyle = grad;
        mapCtx.beginPath(); mapCtx.arc(px, py, 22, 0, Math.PI * 2); mapCtx.fill();

        // Point
        mapCtx.fillStyle = col;
        mapCtx.beginPath(); mapCtx.arc(px, py, 6, 0, Math.PI * 2); mapCtx.fill();

        // Nom du drone
        mapCtx.fillStyle = '#fff';
        mapCtx.font = '600 11px Inter';
        mapCtx.textAlign = 'center';
        mapCtx.fillText(`D${did}`, px, py - 12);

        // Altitude z (m)
        mapCtx.fillStyle = 'rgba(255,255,255,.4)';
        mapCtx.font = '10px JetBrains Mono';
        mapCtx.fillText(`z=${p.z?.toFixed(1) || '?'}`, px, py + 18);
    });
}

// ─────────────────────────────────────────────────────────────
// ÉTIQUETTE DE QUALITÉ DU SIGNAL
// ─────────────────────────────────────────────────────────────

// Renvoie l'étiquette HTML de qualité : > -50 dBm Excellent, > -60 Good, > -70 Fair, sinon Weak (BLOCKED si pas de signal).
function qualityBadge(rssi) {
    if (rssi === null || rssi === 'blocked' || rssi === 'error' || rssi === 'BLOCKED') {
        return '<span class="quality blocked">BLOCKED</span>';
    }
    const v = parseFloat(rssi);
    if (isNaN(v)) return '<span class="quality blocked">N/A</span>';
    if (v > -50) return '<span class="quality excellent">Excellent</span>';
    if (v > -60) return '<span class="quality good">Good</span>';
    if (v > -70) return '<span class="quality fair">Fair</span>';
    return '<span class="quality weak">Weak</span>';
}

// ─────────────────────────────────────────────────────────────
// MISE À JOUR DES ONGLETS RÉSEAU
// ─────────────────────────────────────────────────────────────

// Met à jour les six cartes chiffrées : nombre de drones, RSSI et latence moyens en WiFi et en 5G, tick.
function updateKPIs(wifi, fiveg) {
    // Nombre de drones distincts vus dans les mesures
    let droneSet = new Set();
    if (wifi?.pairs) wifi.pairs.forEach(p => { droneSet.add(p.drone_a); droneSet.add(p.drone_b); });
    if (fiveg?.drones) fiveg.drones.forEach(d => droneSet.add(d.id));
    document.getElementById('valDrones').textContent = droneSet.size || '0';

    // RSSI WiFi moyen (paires bloquées exclues)
    if (wifi?.pairs?.length) {
        const valid = wifi.pairs.filter(p => p.rssi !== 'blocked' && p.rssi !== 'error').map(p => parseFloat(p.rssi));
        const avg = valid.length ? (valid.reduce((a, b) => a + b, 0) / valid.length) : null;
        document.getElementById('valRssiWifi').textContent = avg !== null ? avg.toFixed(1) + ' dBm' : '-- dBm';
    }

    // Latence WiFi moyenne
    if (wifi?.pairs?.length) {
        const valid = wifi.pairs.filter(p => p.latency !== '---').map(p => parseFloat(p.latency));
        const avg = valid.length ? (valid.reduce((a, b) => a + b, 0) / valid.length) : null;
        document.getElementById('valLatWifi').textContent = avg !== null ? avg.toFixed(2) + ' ms' : '-- ms';
    }

    // RSSI 5G moyen (drone → antenne)
    if (fiveg?.drones?.length) {
        const valid = fiveg.drones.filter(d => d.rssi !== null && d.rssi !== 'BLOCKED').map(d => parseFloat(d.rssi));
        const avg = valid.length ? (valid.reduce((a, b) => a + b, 0) / valid.length) : null;
        document.getElementById('valRssi5g').textContent = avg !== null ? avg.toFixed(1) + ' dBm' : '-- dBm';
    }

    // Latence 5G moyenne (entre paires de drones)
    if (fiveg?.pairs?.length) {
        const valid = fiveg.pairs.map(p => parseFloat(p.latency));
        const avg = valid.length ? (valid.reduce((a, b) => a + b, 0) / valid.length) : null;
        document.getElementById('valLat5g').textContent = avg !== null ? avg.toFixed(2) + ' ms' : '-- ms';
    }

    document.getElementById('valTick').textContent = state.tick;
}

// Remplit le tableau WiFi : une ligne par paire (RSSI, latence, distance, qualité).
function updateWifiTable(wifi) {
    const tbody = document.querySelector('#tableWifi tbody');
    if (!wifi?.pairs?.length) {
        tbody.innerHTML = '<tr><td colspan="5" style="text-align:center;color:var(--text-2)">En attente...</td></tr>';
        return;
    }
    tbody.innerHTML = wifi.pairs.map(p => {
        const rssiStr = (p.rssi === 'blocked' || p.rssi === 'error')
            ? '<span style="color:var(--accent-red)">BLOCKED</span>'
            : parseFloat(p.rssi).toFixed(1) + ' dBm';
        const latStr = p.latency === '---' ? '---' : parseFloat(p.latency).toFixed(2) + ' ms';
        return `<tr>
            <td>D${p.drone_a} ↔ D${p.drone_b}</td>
            <td>${rssiStr}</td>
            <td>${latStr}</td>
            <td>${parseFloat(p.distance).toFixed(1)} m</td>
            <td>${qualityBadge(p.rssi)}</td>
        </tr>`;
    }).join('');
}

// Remplit le tableau 5G : une ligne par drone (RSSI vers l'antenne, latence et gigue de sa première paire, qualité).
function update5gTable(fiveg) {
    const tbody = document.querySelector('#table5g tbody');
    if (!fiveg?.drones?.length) {
        tbody.innerHTML = '<tr><td colspan="5" style="text-align:center;color:var(--text-2)">En attente...</td></tr>';
        return;
    }

    // Une ligne par drone (lien drone → antenne)
    let rows = fiveg.drones.map(d => {
        const rssiStr = (d.rssi === null || d.rssi === 'BLOCKED')
            ? '<span style="color:var(--accent-red)">BLOCKED</span>'
            : parseFloat(d.rssi).toFixed(1) + ' dBm';
        // Latence prise dans la première paire qui contient ce drone
        let lat = '--', jit = '--';
        if (fiveg.pairs) {
            const p = fiveg.pairs.find(p => p.drone_a === d.id || p.drone_b === d.id);
            if (p) { lat = parseFloat(p.latency).toFixed(2) + ' ms'; jit = parseFloat(p.jitter).toFixed(2) + ' ms'; }
        }
        return `<tr>
            <td>D${d.id}</td>
            <td>${rssiStr}</td>
            <td>${lat}</td>
            <td>${jit}</td>
            <td>${qualityBadge(d.rssi)}</td>
        </tr>`;
    });
    tbody.innerHTML = rows.join('');
}

// Ajoute les mesures du tick courant à l'historique des courbes (au plus MAX_HISTORY points par série).
function pushHistory(wifi, fiveg) {
    const h = state.history;
    const label = state.tick.toString();
    h.labels.push(label);
    if (h.labels.length > MAX_HISTORY) h.labels.shift();

    // RSSI et latence WiFi, par paire
    if (wifi?.pairs) {
        wifi.pairs.forEach(p => {
            const key = `${p.drone_a}↔${p.drone_b}`;
            if (!h.rssiWifi[key]) h.rssiWifi[key] = [];
            if (!h.latWifi[key]) h.latWifi[key] = [];
            const rssi = (p.rssi === 'blocked' || p.rssi === 'error') ? null : parseFloat(p.rssi);
            const lat = p.latency === '---' ? null : parseFloat(p.latency);
            h.rssiWifi[key].push(rssi);
            h.latWifi[key].push(lat);
            if (h.rssiWifi[key].length > MAX_HISTORY) h.rssiWifi[key].shift();
            if (h.latWifi[key].length > MAX_HISTORY) h.latWifi[key].shift();
        });
    }

    // RSSI 5G, par drone
    if (fiveg?.drones) {
        fiveg.drones.forEach(d => {
            const key = `D${d.id}→gNB`;
            if (!h.rssi5g[key]) h.rssi5g[key] = [];
            const rssi = (d.rssi === null || d.rssi === 'BLOCKED') ? null : parseFloat(d.rssi);
            h.rssi5g[key].push(rssi);
            if (h.rssi5g[key].length > MAX_HISTORY) h.rssi5g[key].shift();
        });
    }

    // Latence 5G, par paire
    if (fiveg?.pairs) {
        fiveg.pairs.forEach(p => {
            const key = `${p.drone_a}↔${p.drone_b} 5G`;
            if (!h.lat5g[key]) h.lat5g[key] = [];
            h.lat5g[key].push(parseFloat(p.latency));
            if (h.lat5g[key].length > MAX_HISTORY) h.lat5g[key].shift();
        });
    }
}

// Redessine les deux courbes dans le temps (RSSI et latence) avec les séries de l'onglet actif (WiFi plein, 5G en tirets).
function updateTimeCharts() {
    const h = state.history;

    const showWifi = (activeScenario === 'both' || activeScenario === 'wifi');
    const show5g = (activeScenario === 'both' || activeScenario === '5g');

    // Courbe RSSI : paires WiFi + drones 5G (selon l'onglet)
    let datasets = [];
    let idx = 0;
    if (showWifi) {
        Object.entries(h.rssiWifi).forEach(([key, data]) => {
            const c = PAIR_COLORS[idx % PAIR_COLORS.length];
            datasets.push({
                label: `WiFi ${key}`,
                data: [...data],
                borderColor: c.wifi,
                backgroundColor: c.wifi + '22',
                borderWidth: 2,
                tension: 0.3,
                fill: false,
            });
            idx++;
        });
    }
    if (show5g) {
        Object.entries(h.rssi5g).forEach(([key, data]) => {
            const c = PAIR_COLORS[idx % PAIR_COLORS.length];
            datasets.push({
                label: `5G ${key}`,
                data: [...data],
                borderColor: c.fiveg,
                backgroundColor: c.fiveg + '22',
                borderWidth: 2,
                borderDash: [5, 3],
                tension: 0.3,
                fill: false,
            });
            idx++;
        });
    }
    chartRSSI.data.labels = [...h.labels];
    chartRSSI.data.datasets = datasets;
    chartRSSI.update('none');

    // Courbe latence : paires WiFi + paires 5G (selon l'onglet)
    datasets = [];
    idx = 0;
    if (showWifi) {
        Object.entries(h.latWifi).forEach(([key, data]) => {
            const c = PAIR_COLORS[idx % PAIR_COLORS.length];
            datasets.push({
                label: `WiFi ${key}`,
                data: [...data],
                borderColor: c.wifi,
                backgroundColor: c.wifi + '22',
                borderWidth: 2,
                tension: 0.3,
                fill: false,
            });
            idx++;
        });
    }
    if (show5g) {
        Object.entries(h.lat5g).forEach(([key, data]) => {
            const c = PAIR_COLORS[idx % PAIR_COLORS.length];
            datasets.push({
                label: `5G ${key}`,
                data: [...data],
                borderColor: c.fiveg,
                backgroundColor: c.fiveg + '22',
                borderWidth: 2,
                borderDash: [5, 3],
                tension: 0.3,
                fill: false,
            });
            idx++;
        });
    }
    chartLatency.data.labels = [...h.labels];
    chartLatency.data.datasets = datasets;
    chartLatency.update('none');
}

// Redessine les trois graphiques en barres : RSSI WiFi / 5G, latence WiFi / 5G, distance de chaque paire.
function updateComparisonCharts(wifi, fiveg) {
    // Barres RSSI WiFi / 5G par paire
    const pairLabels = [];
    const wifiRssi = [];
    const fivegRssi = [];

    if (wifi?.pairs) {
        wifi.pairs.forEach(p => {
            pairLabels.push(`D${p.drone_a}↔D${p.drone_b}`);
            const r = (p.rssi === 'blocked' || p.rssi === 'error') ? null : parseFloat(p.rssi);
            wifiRssi.push(r);
        });
    }

    // En 5G, chaque drone parle à l'antenne : le RSSI d'une paire est la moyenne de ses deux drones
    if (fiveg?.pairs && fiveg?.drones) {
        const droneRssi = {};
        fiveg.drones.forEach(d => { droneRssi[d.id] = d.rssi; });
        // Même ordre de paires que le WiFi
        if (wifi?.pairs) {
            wifi.pairs.forEach(p => {
                const ra = droneRssi[p.drone_a], rb = droneRssi[p.drone_b];
                if (ra != null && rb != null && ra !== 'BLOCKED' && rb !== 'BLOCKED') {
                    fivegRssi.push((parseFloat(ra) + parseFloat(rb)) / 2);
                } else {
                    fivegRssi.push(null);
                }
            });
        }
    }

    chartCompRSSI.data.labels = pairLabels;
    chartCompRSSI.data.datasets = [
        { label: 'WiFi', data: wifiRssi, backgroundColor: C.wifiBg, borderColor: C.wifi, borderWidth: 2, borderRadius: 6 },
        { label: '5G NR', data: fivegRssi, backgroundColor: C.fivegBg, borderColor: C.fiveg, borderWidth: 2, borderRadius: 6 },
    ];
    chartCompRSSI.update('none');

    // Barres de latence WiFi / 5G par paire
    const latLabels = [];
    const wifiLat = [];
    const fivegLat = [];

    if (wifi?.pairs) {
        wifi.pairs.forEach(p => {
            latLabels.push(`D${p.drone_a}↔D${p.drone_b}`);
            wifiLat.push(p.latency === '---' ? null : parseFloat(p.latency));
        });
    }
    if (fiveg?.pairs) {
        fiveg.pairs.forEach((p, i) => {
            if (i < latLabels.length) fivegLat.push(parseFloat(p.latency));
        });
    }

    chartCompLat.data.labels = latLabels;
    chartCompLat.data.datasets = [
        { label: 'WiFi', data: wifiLat, backgroundColor: C.wifiBg, borderColor: C.wifi, borderWidth: 2, borderRadius: 6 },
        { label: '5G NR', data: fivegLat, backgroundColor: C.fivegBg, borderColor: C.fiveg, borderWidth: 2, borderRadius: 6 },
    ];
    chartCompLat.update('none');

    // Barres de distance entre drones (paires WiFi)
    const distLabels = [];
    const distVals = [];
    if (wifi?.pairs) {
        wifi.pairs.forEach(p => {
            distLabels.push(`D${p.drone_a}↔D${p.drone_b}`);
            distVals.push(parseFloat(p.distance));
        });
    }
    chartDist.data.labels = distLabels;
    chartDist.data.datasets = [{
        label: 'Distance',
        data: distVals,
        backgroundColor: C.greenBg,
        borderColor: C.green,
        borderWidth: 2,
        borderRadius: 6,
    }];
    chartDist.update('none');
}

// ─────────────────────────────────────────────────────────────
// ONGLET EXPLORATION — DESSINS SUR CANVAS
// ─────────────────────────────────────────────────────────────

// Couleurs et noms des 3 drones de l'onglet Exploration.
const DRONE_COLS = ['#22d3ee', '#34d399', '#a78bfa'];
const DRONE_NAMES = ['Drone 0', 'Drone 1', 'Drone 2'];

// Couleur et libellé de chaque état de drone (en vol, crash, récupération, posé).
const STATUS_COLORS = {
    flying: '#4caf50',
    crashed: '#f44336',
    recovering: '#ff9800',
    landed: '#78909c',
};
const STATUS_LABELS = {
    flying: 'En vol',
    crashed: 'CRASH',
    recovering: 'Récupération',
    landed: 'Posé',
};

// Renvoie la couleur d'une case : sombre si jamais vue, rouge / brun si p > 0,6 (occupée), vert si p < 0,4 (libre), gris sinon.
function occColor(p, visited) {
    if (!visited) return '#111827';
    if (p > 0.75) return '#b91c1c';
    if (p > 0.6) return '#92400e';
    if (p < 0.25) return '#065f46';
    if (p < 0.4) return '#047857';
    return '#374151';
}

// ── Conversion mètres → pixels pour une grille donnée ──
// Renvoie {toX, toY} qui placent un point (m) de la grille g dans un canvas de W × H pixels.
function makeCoord(g, W, H) {
    return {
        // Abscisse x (m) → pixel horizontal.
        toX: (wx) => ((wx - g.origin_x) / (g.width * g.resolution)) * W,
        // Ordonnée y (m) → pixel vertical (axe inversé : y vers le haut).
        toY: (wy) => H - ((wy - g.origin_y) / (g.height * g.resolution)) * H,
    };
}

// ═══════════════════════════════════════════════════════════
// GRANDE CARTE — grille d'occupation complète + drones + traces + cibles
// ═══════════════════════════════════════════════════════════
// Dessine la grande carte : grille d'occupation, frontières, obstacles connus, traces, portée lidar et drones.
function drawHeroMap(expl) {
    if (!heroCtx || !heroCanvas || !expl || !expl.grid) return;
    const g = expl.grid;
    const W = heroCanvas.width, H = heroCanvas.height;
    if (W === 0 || H === 0) return;
    heroCtx.clearRect(0, 0, W, H);

    const cellW = W / g.width;
    const cellH = H / g.height;

    // Grille d'occupation (ligne 0 de la grille en bas du canvas)
    for (let gy = 0; gy < g.height; gy++) {
        for (let gx = 0; gx < g.width; gx++) {
            const idx = gy * g.width + gx;
            const p = g.data[idx];
            const v = g.visited ? g.visited[idx] : (Math.abs(p - 0.5) > 0.02);
            heroCtx.fillStyle = occColor(p, v);
            heroCtx.fillRect(gx * cellW, (g.height - 1 - gy) * cellH, cellW + 0.5, cellH + 0.5);
        }
    }

    // Frontières en jaune (limite entre zone connue et zone inconnue)
    if (expl.frontiers) {
        heroCtx.fillStyle = 'rgba(234,179,8,0.5)';
        expl.frontiers.forEach(([gx, gy]) => {
            heroCtx.fillRect(gx * cellW, (g.height - 1 - gy) * cellH, cellW + 0.5, cellH + 0.5);
        });
    }

    const { toX, toY } = makeCoord(g, W, H);

    // Cadre = bords de la grille d'exploration reçue
    const minX = g.origin_x;
    const maxX = g.origin_x + g.width * g.resolution;
    const minY = g.origin_y;
    const maxY = g.origin_y + g.height * g.resolution;
    heroCtx.strokeStyle = 'rgba(255,255,255,0.12)';
    heroCtx.lineWidth = 1.5;
    heroCtx.strokeRect(toX(minX), toY(maxY), toX(maxX) - toX(minX), toY(minY) - toY(maxY));

    // Obstacles connus envoyés par le serveur (boîtes et cylindres), s'il y en a
    const envObs = (expl.environment && Array.isArray(expl.environment.obstacles))
        ? expl.environment.obstacles
        : [];
    if (envObs.length) {
        const sx = W / (g.width * g.resolution);
        const sy = H / (g.height * g.resolution);
        heroCtx.fillStyle = 'rgba(148,163,184,0.16)';
        heroCtx.strokeStyle = 'rgba(148,163,184,0.45)';
        heroCtx.lineWidth = 1;

        envObs.forEach((obs) => {
            if (obs.type === 'box' && Number.isFinite(obs.x) && Number.isFinite(obs.y) && Number.isFinite(obs.w) && Number.isFinite(obs.h)) {
                const rx = toX(obs.x);
                const ry = toY(obs.y + obs.h);
                const rw = obs.w * sx;
                const rh = obs.h * sy;
                heroCtx.fillRect(rx, ry, rw, rh);
                heroCtx.strokeRect(rx, ry, rw, rh);
            } else if (obs.type === 'cylinder' && Number.isFinite(obs.x) && Number.isFinite(obs.y) && Number.isFinite(obs.radius)) {
                const px = toX(obs.x);
                const py = toY(obs.y);
                const rr = obs.radius * Math.min(sx, sy);
                heroCtx.beginPath();
                heroCtx.arc(px, py, rr, 0, Math.PI * 2);
                heroCtx.fill();
                heroCtx.stroke();
            }
        });
    }

    // Traces des drones (plus pâles pour les points anciens)
    if (expl.trajectories) {
        Object.entries(expl.trajectories).forEach(([did, pts], idx) => {
            if (pts.length < 2) return;
            const col = DRONE_COLS[idx % 3];
            for (let i = 1; i < pts.length; i++) {
                const alpha = 0.15 + 0.65 * (i / pts.length);
                heroCtx.strokeStyle = col + Math.round(alpha * 255).toString(16).padStart(2, '0');
                heroCtx.lineWidth = 2.5;
                heroCtx.beginPath();
                heroCtx.moveTo(toX(pts[i - 1][0]), toY(pts[i - 1][1]));
                heroCtx.lineTo(toX(pts[i][0]), toY(pts[i][1]));
                heroCtx.stroke();
            }
        });
    }

    // Drones
    if (expl.drones) {
        expl.drones.forEach((d, idx) => {
            const px = toX(d.x), py = toY(d.y);
            const col = DRONE_COLS[idx % 3];

            // Portée du lidar (capteur laser de distance) : cercle pointillé, 8 m par défaut
            const lidarRange = (expl.environment && Number(expl.environment.lidar_max_range)) || 8.0;
            const lidarR = lidarRange * Math.min(
                W / (g.width * g.resolution),
                H / (g.height * g.resolution)
            );
            heroCtx.strokeStyle = col + '25';
            heroCtx.lineWidth = 1;
            heroCtx.setLineDash([3, 3]);
            heroCtx.beginPath(); heroCtx.arc(px, py, lidarR, 0, Math.PI * 2); heroCtx.stroke();
            heroCtx.setLineDash([]);

            // Flèche pointillée vers la cible
            if (d.target_x != null && d.target_y != null) {
                const tx = toX(d.target_x), ty = toY(d.target_y);
                heroCtx.strokeStyle = col + 'aa';
                heroCtx.lineWidth = 2;
                heroCtx.setLineDash([6, 4]);
                heroCtx.beginPath(); heroCtx.moveTo(px, py); heroCtx.lineTo(tx, ty); heroCtx.stroke();
                heroCtx.setLineDash([]);
                // Viseur sur la cible
                heroCtx.strokeStyle = '#4ade80';
                heroCtx.lineWidth = 2;
                heroCtx.beginPath();
                heroCtx.arc(tx, ty, 6, 0, Math.PI * 2);
                heroCtx.stroke();
                heroCtx.beginPath(); heroCtx.moveTo(tx - 9, ty); heroCtx.lineTo(tx + 9, ty); heroCtx.stroke();
                heroCtx.beginPath(); heroCtx.moveTo(tx, ty - 9); heroCtx.lineTo(tx, ty + 9); heroCtx.stroke();
            }

            // Anneau rouge si le drone s'est écrasé
            if (d.status === 'crashed') {
                heroCtx.strokeStyle = '#f44336';
                heroCtx.lineWidth = 3;
                heroCtx.beginPath(); heroCtx.arc(px, py, 18, 0, Math.PI * 2); heroCtx.stroke();
                heroCtx.fillStyle = 'rgba(244,67,54,0.12)';
                heroCtx.beginPath(); heroCtx.arc(px, py, 18, 0, Math.PI * 2); heroCtx.fill();
            }

            // Halo
            const grad = heroCtx.createRadialGradient(px, py, 0, px, py, 20);
            grad.addColorStop(0, col + '55');
            grad.addColorStop(1, col + '00');
            heroCtx.fillStyle = grad;
            heroCtx.beginPath(); heroCtx.arc(px, py, 20, 0, Math.PI * 2); heroCtx.fill();

            // Icône du drone : triangle orienté selon son cap (yaw, en radians)
            const yaw = d.yaw || 0;
            heroCtx.save();
            heroCtx.translate(px, py);
            heroCtx.rotate(-yaw + Math.PI / 2);
            heroCtx.fillStyle = d.status === 'crashed' ? '#f44336' : col;
            heroCtx.beginPath();
            heroCtx.moveTo(0, -9);
            heroCtx.lineTo(-6, 7);
            heroCtx.lineTo(6, 7);
            heroCtx.closePath();
            heroCtx.fill();
            heroCtx.restore();

            // Nom du drone
            heroCtx.fillStyle = '#fff';
            heroCtx.font = '700 12px Inter';
            heroCtx.textAlign = 'center';
            heroCtx.fillText(`D${d.id}`, px, py - 16);

            // Altitude z (m) et vitesse v (m/s)
            heroCtx.fillStyle = 'rgba(255,255,255,.5)';
            heroCtx.font = '10px JetBrains Mono';
            heroCtx.fillText(`z=${d.z?.toFixed(1)}  v=${(d.speed || 0).toFixed(1)}`, px, py + 24);
        });
    }
}

// ═══════════════════════════════════════════════════════════
// CARTE PAR DRONE — la même grille, centrée sur ce que voit chaque drone
// ═══════════════════════════════════════════════════════════
// Dessine le panneau d'un drone : grille assombrie hors portée lidar, trace, cible, puis met à jour état et infos.
function drawDroneBelief(ctx, canvas, expl, droneIdx) {
    if (!ctx || !canvas || !expl || !expl.grid) return;
    const g = expl.grid;
    const W = canvas.width, H = canvas.height;
    if (W === 0 || H === 0) return;
    ctx.clearRect(0, 0, W, H);

    const d = expl.drones ? expl.drones[droneIdx] : null;
    if (!d || d.x == null || d.y == null) return;

    const col = DRONE_COLS[droneIdx % 3];
    const cellW = W / g.width;
    const cellH = H / g.height;

    // Grille complète d'abord ; la zone hors portée lidar est assombrie juste après
    for (let gy = 0; gy < g.height; gy++) {
        for (let gx = 0; gx < g.width; gx++) {
            const idx = gy * g.width + gx;
            const p = g.data[idx];
            const v = g.visited ? g.visited[idx] : (Math.abs(p - 0.5) > 0.02);
            ctx.fillStyle = occColor(p, v);
            ctx.fillRect(gx * cellW, (g.height - 1 - gy) * cellH, cellW + 0.5, cellH + 0.5);
        }
    }

    // Voile sombre partout sauf dans le disque de portée du lidar
    const { toX, toY } = makeCoord(g, W, H);
    const px = toX(d.x), py = toY(d.y);
    const lidarRange = (expl.environment && Number(expl.environment.lidar_max_range)) || 8.0;
    const lidarR = lidarRange * Math.min(
        W / (g.width * g.resolution),
        H / (g.height * g.resolution)
    );

    ctx.save();
    ctx.fillStyle = 'rgba(0,0,0,0.55)';
    ctx.beginPath();
    ctx.rect(0, 0, W, H);
    ctx.arc(px, py, lidarR, 0, Math.PI * 2, true); // cut out circle
    ctx.fill();
    ctx.restore();

    // Bord du disque lidar
    ctx.strokeStyle = col + '60';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(px, py, lidarR, 0, Math.PI * 2); ctx.stroke();

    // Trace de ce drone seulement
    if (expl.trajectories) {
        const pts = expl.trajectories[droneIdx] || expl.trajectories[String(droneIdx)];
        if (pts && pts.length > 1) {
            ctx.strokeStyle = col + '66';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(toX(pts[0][0]), toY(pts[0][1]));
            for (let i = 1; i < pts.length; i++) {
                ctx.lineTo(toX(pts[i][0]), toY(pts[i][1]));
            }
            ctx.stroke();
        }
    }

    // Cible
    if (d.target_x != null && d.target_y != null) {
        const tx = toX(d.target_x), ty = toY(d.target_y);
        ctx.strokeStyle = '#4ade80';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 3]);
        ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(tx, ty); ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath(); ctx.arc(tx, ty, 5, 0, Math.PI * 2); ctx.stroke();
    }

    // Icône du drone (triangle orienté selon son cap)
    const yaw = d.yaw || 0;
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-yaw + Math.PI / 2);
    ctx.fillStyle = d.status === 'crashed' ? '#f44336' : col;
    ctx.beginPath();
    ctx.moveTo(0, -8);
    ctx.lineTo(-5, 6);
    ctx.lineTo(5, 6);
    ctx.closePath();
    ctx.fill();
    ctx.restore();

    // Étiquette d'état dans l'en-tête du panneau
    const statusTag = document.getElementById(`statusDrone${droneIdx}`);
    if (statusTag) {
        const sc = STATUS_COLORS[d.status] || col;
        statusTag.textContent = STATUS_LABELS[d.status] || d.status;
        statusTag.style.color = sc;
        statusTag.style.background = sc + '22';
    }

    // Barre d'infos : position, altitude, vitesse, cible, cellules découvertes, distance parcourue
    const infoBar = document.getElementById(`infoDrone${droneIdx}`);
    if (infoBar) {
        const tgt = (d.target_x != null) ? `(${d.target_x.toFixed(1)}, ${d.target_y.toFixed(1)})` : '—';
        infoBar.innerHTML =
            `<span class="di-item"><b>Pos</b> (${d.x.toFixed(1)}, ${d.y.toFixed(1)})</span>` +
            `<span class="di-item"><b>Alt</b> ${d.z.toFixed(1)}m</span>` +
            `<span class="di-item"><b>Vit</b> ${(d.speed || 0).toFixed(1)} m/s</span>` +
            `<span class="di-item"><b>🎯</b> ${tgt}</span>` +
            `<span class="di-item"><b>Cellules</b> ${d.cells_discovered}</span>` +
            `<span class="di-item"><b>Dist</b> ${d.distance_traveled.toFixed(1)}m</span>`;
    }

    // Panneau qui clignote en rouge si crash, bord orange pendant la récupération
    const panel = document.getElementById(`panelDrone${droneIdx}`);
    if (panel) {
        panel.classList.toggle('panel-crashed', d.status === 'crashed');
        panel.classList.toggle('panel-recovering', d.status === 'recovering');
    }
}

// ═══════════════════════════════════════════════════════════
// JAUGES
// ═══════════════════════════════════════════════════════════

// Dessine la jauge circulaire du pourcentage exploré, avec le numéro de step et le nombre de cases vues.
function drawGaugeProgress(expl) {
    if (!gaugeProgressCtx || !gaugeProgressCanvas) return;
    const W = gaugeProgressCanvas.width, H = gaugeProgressCanvas.height;
    if (W === 0 || H === 0) return;
    gaugeProgressCtx.clearRect(0, 0, W, H);
    const ctx = gaugeProgressCtx;

    const pct = expl ? (expl.explored_pct || 0) : 0;
    const cx = W / 2, cy = H / 2;
    const r = Math.min(cx, cy) - 20;

    // Arc de fond (270°)
    ctx.strokeStyle = '#1f2937';
    ctx.lineWidth = 14;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0.75 * Math.PI, 2.25 * Math.PI);
    ctx.stroke();

    // Arc de progression, proportionnel au pourcentage exploré
    const endAngle = 0.75 * Math.PI + (pct / 100) * 1.5 * Math.PI;
    const grad = ctx.createLinearGradient(cx - r, cy, cx + r, cy);
    grad.addColorStop(0, '#065f46');
    grad.addColorStop(1, '#4ade80');
    ctx.strokeStyle = grad;
    ctx.lineWidth = 14;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0.75 * Math.PI, endAngle);
    ctx.stroke();

    // Pourcentage au centre
    ctx.fillStyle = '#fff';
    ctx.font = '700 32px Inter';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${pct.toFixed(0)}%`, cx, cy - 8);

    ctx.fillStyle = 'rgba(255,255,255,.5)';
    ctx.font = '12px Inter';
    ctx.fillText('exploré', cx, cy + 18);

    // Step et cases vues / cases totales
    if (expl) {
        ctx.fillStyle = 'rgba(255,255,255,.4)';
        ctx.font = '11px JetBrains Mono';
        ctx.fillText(`Step ${expl.step || 0}  •  ${expl.explored_cells || 0}/${expl.total_cells || 0} cells`, cx, cy + r + 10);
    }
}

// Dessine une barre d'altitude par drone (échelle 0-8 m), avec les lignes « danger 1 m » et « cible 4 m ».
function drawGaugeAltitude(expl) {
    if (!gaugeAltCtx || !gaugeAltCanvas) return;
    const W = gaugeAltCanvas.width, H = gaugeAltCanvas.height;
    if (W === 0 || H === 0) return;
    gaugeAltCtx.clearRect(0, 0, W, H);
    const ctx = gaugeAltCtx;

    if (!expl || !expl.drones || !expl.drones.length) return;

    const maxAlt = 8;
    const barW = 40;
    const gap = 24;
    const totalW = expl.drones.length * (barW + gap) - gap;
    const startX = (W - totalW) / 2;
    const barTop = 30;
    const barBot = H - 40;
    const barH = barBot - barTop;
    const dangerY = barBot - (1.0 / maxAlt) * barH;
    const targetY = barBot - (4.0 / maxAlt) * barH;

    // Ligne de danger à 1 m
    ctx.strokeStyle = 'rgba(244,67,54,0.4)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 3]);
    ctx.beginPath(); ctx.moveTo(startX - 10, dangerY); ctx.lineTo(startX + totalW + 10, dangerY); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(244,67,54,0.5)';
    ctx.font = '9px Inter';
    ctx.textAlign = 'left';
    ctx.fillText('danger 1m', startX + totalW + 14, dangerY + 3);

    // Ligne d'altitude cible à 4 m
    ctx.strokeStyle = 'rgba(74,222,128,0.3)';
    ctx.setLineDash([4, 3]);
    ctx.beginPath(); ctx.moveTo(startX - 10, targetY); ctx.lineTo(startX + totalW + 10, targetY); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(74,222,128,0.4)';
    ctx.fillText('cible 4m', startX + totalW + 14, targetY + 3);

    expl.drones.forEach((d, idx) => {
        const x = startX + idx * (barW + gap);
        const col = DRONE_COLS[idx % 3];
        const alt = d.z || 0;
        const fillH = Math.min((alt / maxAlt) * barH, barH);

        // Fond de la barre
        ctx.fillStyle = '#1f2937';
        ctx.beginPath();
        ctx.roundRect(x, barTop, barW, barH, 6);
        ctx.fill();

        // Remplissage (rouge sous 1,5 m)
        const isDanger = alt < 1.5;
        const barGrad = ctx.createLinearGradient(x, barBot, x, barBot - fillH);
        barGrad.addColorStop(0, isDanger ? '#f44336' : col);
        barGrad.addColorStop(1, isDanger ? '#f4433688' : col + '88');
        ctx.fillStyle = barGrad;
        ctx.beginPath();
        ctx.roundRect(x, barBot - fillH, barW, fillH, 6);
        ctx.fill();

        // Valeur (m)
        ctx.fillStyle = '#fff';
        ctx.font = '700 14px JetBrains Mono';
        ctx.textAlign = 'center';
        ctx.fillText(`${alt.toFixed(1)}`, x + barW / 2, barBot - fillH - 8);

        // Nom du drone
        ctx.fillStyle = col;
        ctx.font = '600 11px Inter';
        ctx.fillText(`D${d.id}`, x + barW / 2, barBot + 18);
    });
}

// Dessine un arc de vitesse par drone (échelle 0-10 m/s) et la distance parcourue par chacun (m).
function drawGaugeSpeed(expl) {
    if (!gaugeSpeedCtx || !gaugeSpeedCanvas) return;
    const W = gaugeSpeedCanvas.width, H = gaugeSpeedCanvas.height;
    if (W === 0 || H === 0) return;
    gaugeSpeedCtx.clearRect(0, 0, W, H);
    const ctx = gaugeSpeedCtx;

    if (!expl || !expl.drones || !expl.drones.length) return;

    const cx = W / 2;
    const meterR = Math.min(W, H) / 2 - 30;
    const cy = H / 2 + 10;
    const maxSpeed = 10;

    // Un arc par drone, emboîtés (16 px d'écart)
    expl.drones.forEach((d, idx) => {
        const col = DRONE_COLS[idx % 3];
        const speed = d.speed || 0;
        const r = meterR - idx * 16;
        const startA = 0.8 * Math.PI;
        const endA = startA + (Math.min(speed, maxSpeed) / maxSpeed) * 1.4 * Math.PI;

        // Arc de fond
        ctx.strokeStyle = '#1f2937';
        ctx.lineWidth = 10;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0.8 * Math.PI, 2.2 * Math.PI);
        ctx.stroke();

        // Arc de vitesse
        ctx.strokeStyle = col;
        ctx.lineWidth = 10;
        ctx.beginPath();
        ctx.arc(cx, cy, r, startA, endA);
        ctx.stroke();

        // Vitesse en texte (m/s)
        ctx.fillStyle = col;
        ctx.font = '600 12px JetBrains Mono';
        ctx.textAlign = 'center';
        ctx.fillText(`D${d.id}: ${speed.toFixed(1)} m/s`, cx, cy + meterR + 12 + idx * 16);
    });

    // Distance parcourue par chaque drone (m)
    ctx.fillStyle = 'rgba(255,255,255,.4)';
    ctx.font = '10px Inter';
    ctx.textAlign = 'center';
    expl.drones.forEach((d, idx) => {
        const dt = d.distance_traveled != null ? d.distance_traveled.toFixed(0) : '0';
        ctx.fillText(`${dt}m parcourus`, cx, cy - meterR + 8 + idx * 14);
    });
}

// Dessine le schéma des distances entre drones (en m), en rouge quand deux drones sont à moins de 3 m.
function drawGaugeDist(expl) {
    if (!gaugeDistCtx || !gaugeDistCanvas) return;
    const W = gaugeDistCanvas.width, H = gaugeDistCanvas.height;
    if (W === 0 || H === 0) return;
    gaugeDistCtx.clearRect(0, 0, W, H);
    const ctx = gaugeDistCtx;

    if (!expl || !expl.drones || expl.drones.length < 2) return;

    const pairs = [];
    for (let i = 0; i < expl.drones.length; i++) {
        for (let j = i + 1; j < expl.drones.length; j++) {
            const a = expl.drones[i], b = expl.drones[j];
            const dist = Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2);
            pairs.push({ a: i, b: j, dist });
        }
    }

    // Drones placés sur un cercle (triangle pour 3 drones) : schéma, pas leurs vraies positions
    const cx = W / 2, cy = H / 2;
    const r = Math.min(W, H) / 2 - 50;
    const positions = expl.drones.map((d, i) => {
        const angle = -Math.PI / 2 + (i * 2 * Math.PI / expl.drones.length);
        return { x: cx + r * Math.cos(angle), y: cy + r * Math.sin(angle) };
    });

    // Segments entre drones (rouges sous 3 m)
    pairs.forEach(({ a, b, dist }) => {
        const pa = positions[a], pb = positions[b];
        const tooClose = dist < 3;
        ctx.strokeStyle = tooClose ? '#f44336' : 'rgba(255,255,255,.15)';
        ctx.lineWidth = tooClose ? 2 : 1;
        ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y); ctx.stroke();

        // Distance écrite au milieu du segment
        const mx = (pa.x + pb.x) / 2, my = (pa.y + pb.y) / 2;
        ctx.fillStyle = tooClose ? '#f44336' : '#fff';
        ctx.font = '700 13px JetBrains Mono';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(`${dist.toFixed(1)}m`, mx, my);
    });

    // Disques des drones
    expl.drones.forEach((d, idx) => {
        const p = positions[idx];
        const col = DRONE_COLS[idx % 3];

        const grad = ctx.createRadialGradient(p.x, p.y, 0, p.x, p.y, 24);
        grad.addColorStop(0, col + '44');
        grad.addColorStop(1, col + '00');
        ctx.fillStyle = grad;
        ctx.beginPath(); ctx.arc(p.x, p.y, 24, 0, Math.PI * 2); ctx.fill();

        ctx.fillStyle = col;
        ctx.beginPath(); ctx.arc(p.x, p.y, 10, 0, Math.PI * 2); ctx.fill();

        ctx.fillStyle = '#fff';
        ctx.font = '700 11px Inter';
        ctx.textAlign = 'center';
        ctx.fillText(`D${d.id}`, p.x, p.y - 18);
    });

    // Rappel du rayon de coordination (6 m)
    ctx.fillStyle = 'rgba(255,255,255,.3)';
    ctx.font = '10px Inter';
    ctx.textAlign = 'center';
    ctx.fillText('Rayon de coordination : 6m', cx, H - 10);
}

// ── Journal des événements de l'exploration ──
// Affiche les 50 derniers événements (crash, alerte, cible, info) selon les cases cochées.
function updateEventTimeline(expl) {
    if (!expl || !expl.events) return;
    const timeline = document.getElementById('eventTimeline');
    const events = expl.events;
    if (!events.length) return;

    const showCrash = document.getElementById('evtFilterCrash').checked;
    const showWarning = document.getElementById('evtFilterWarning').checked;
    const showTarget = document.getElementById('evtFilterTarget').checked;
    const showInfo = document.getElementById('evtFilterInfo').checked;

    const filtered = events.filter(e => {
        if (e.type === 'crash' || e.type === 'recovery') return showCrash;
        if (e.type === 'warning') return showWarning;
        if (e.type === 'target') return showTarget;
        return showInfo;
    });

    const EVT_ICONS = { crash: '🔴', recovery: '🟠', warning: '⚠️', target: '🎯', info: 'ℹ️' };
    const EVT_CLASSES = { crash: 'evt-crash', recovery: 'evt-warning', warning: 'evt-warning', target: 'evt-target', info: 'evt-info' };

    const recent = filtered.slice(-50).reverse();
    timeline.innerHTML = recent.map(e => {
        const icon = EVT_ICONS[e.type] || 'ℹ️';
        const cls = EVT_CLASSES[e.type] || 'evt-info';
        const col = DRONE_COLS[e.drone % 3];
        return `<div class="evt-entry ${cls}">
            <span class="evt-icon">${icon}</span>
            <span class="evt-step">S${e.step}</span>
            <span class="evt-time">${e.time}</span>
            <span class="evt-drone" style="color:${col}">D${e.drone}</span>
            <span class="evt-msg">${e.message}</span>
        </div>`;
    }).join('');
}

// Bouton « Effacer » du journal des événements : vide la liste affichée.
document.getElementById('btnClearEvents').addEventListener('click', () => {
    document.getElementById('eventTimeline').innerHTML =
        '<div class="event-empty">Journal effacé.</div>';
});

// Met à jour tout l'onglet Exploration : en-tête (step, %, état « Terminée » dès 95 %), cartes, jauges, journal.
function updateExploration(expl) {
    if (!expl) return;

    // Chiffres de l'en-tête de la grande carte
    const heroStep = document.getElementById('heroStep');
    const heroPct = document.getElementById('heroPct');
    const heroStat = document.getElementById('heroStatus');
    if (heroStep) heroStep.textContent = `Step ${expl.step || 0}`;
    if (heroPct) heroPct.textContent = `${(expl.explored_pct || 0).toFixed(1)}%`;
    if (heroStat) {
        const pct = expl.explored_pct || 0;
        const crashed = expl.drones ? expl.drones.filter(d => d.status === 'crashed').length : 0;
        if (pct >= 95) {
            heroStat.textContent = 'Terminée ✓';
            heroStat.classList.add('explore-done');
        } else if (crashed > 0) {
            heroStat.textContent = `⚠ ${crashed} crash`;
            heroStat.classList.add('explore-warning');
        } else {
            heroStat.textContent = `En cours…`;
            heroStat.classList.remove('explore-done', 'explore-warning');
        }
    }

    // Redessine tous les canvas de l'onglet
    drawHeroMap(expl);
    for (let i = 0; i < 3; i++) {
        drawDroneBelief(droneCtxs[i], droneCanvases[i], expl, i);
    }
    drawGaugeProgress(expl);
    drawGaugeAltitude(expl);
    drawGaugeSpeed(expl);
    drawGaugeDist(expl);
    updateEventTimeline(expl);
}

// ─────────────────────────────────────────────────────────────
// JOURNAL EN BAS DE PAGE
// ─────────────────────────────────────────────────────────────

const logBody = document.getElementById('logBody');
// Ajoute une ligne horodatée au journal (type : data, info ou error) ; garde les 200 dernières.
function addLog(msg, type = 'data') {
    const el = document.createElement('div');
    el.className = `log-entry ${type}`;
    const now = new Date().toLocaleTimeString('fr-FR');
    el.textContent = `[${now}] ${msg}`;
    logBody.appendChild(el);
    if (logBody.children.length > 200) logBody.removeChild(logBody.firstChild);
    logBody.scrollTop = logBody.scrollHeight;
}

// Bouton « Effacer » du journal : le vide et y note l'effacement.
document.getElementById('btnClearLog').addEventListener('click', () => {
    logBody.innerHTML = '';
    addLog('Journal effacé.', 'info');
});

// ─────────────────────────────────────────────────────────────
// ÉTAT DE LA CONNEXION
// ─────────────────────────────────────────────────────────────

const pill = document.getElementById('statusPill');
// Affiche l'état de la connexion au serveur dans l'en-tête (vert si ok, rouge sinon) avec le texte donné.
function setStatus(ok, text) {
    pill.className = 'status-pill ' + (ok ? 'connected' : 'error');
    pill.querySelector('.status-text').textContent = text;
}

// Horloge de l'en-tête, mise à jour chaque seconde.
setInterval(() => {
    document.getElementById('clock').textContent = new Date().toLocaleTimeString('fr-FR');
}, 1000);

// ─────────────────────────────────────────────────────────────
// ONGLETS — Exploration / Comparaison / WiFi / 5G
// ─────────────────────────────────────────────────────────────

// Un clic sur un onglet le rend actif, applique le filtre correspondant et le note dans le journal.
document.querySelectorAll('.tab').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        applyFilter(btn.dataset.scenario);
        addLog(`Filtre: ${btn.textContent}`, 'info');
    });
});

// ─────────────────────────────────────────────────────────────
// INTERROGATION DU SERVEUR
// ─────────────────────────────────────────────────────────────

// Dernier tick ajouté à l'historique des courbes.
let prevTick = -1;

// Demande /api/data, puis met à jour toute la page ; en cas d'erreur, passe l'état à « Déconnecté ».
async function poll() {
    try {
        const res = await fetch(API + '/data');
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const data = await res.json();

        if (!state.connected) {
            setStatus(true, 'Connecté');
            addLog('Connexion au backend établie.', 'info');
            state.connected = true;
        }

        const wifi = data.wifi;
        const fiveg = data.fiveg;
        const livePositions = data.positions || {};
        state.tick = data.tick || state.tick;
        state.currentWifi = wifi;
        state.current5g = fiveg;
        state.livePositions = livePositions;

        // Historique complété seulement quand le tick change (le tick avance d'au moins 1)
        if (state.tick !== prevTick) {
            state.tick = Math.max(state.tick, prevTick + 1);
            pushHistory(wifi, fiveg);
            prevTick = state.tick;

            if (wifi?.pairs?.length || fiveg?.drones?.length) {
                const nPairs = wifi?.pairs?.length || 0;
                const nDrones5g = fiveg?.drones?.length || 0;
                addLog(`Tick ${state.tick}: WiFi ${nPairs} paires | 5G ${nDrones5g} drones`, 'data');
            }
        }

        updateKPIs(wifi, fiveg);
        updateWifiTable(wifi);
        update5gTable(fiveg);
        updateTimeCharts();
        updateComparisonCharts(wifi, fiveg);
        drawMap(wifi, fiveg, livePositions);

        // Onglet Exploration
        const expl = data.exploration;
        if (expl) {
            updateExploration(expl);
        }

    } catch (err) {
        if (state.connected) {
            setStatus(false, 'Déconnecté');
            addLog(`Erreur: ${err.message}`, 'error');
            state.connected = false;
        }
    }
}

// Démarrage : interrogation toutes les POLL_MS ms (et une fois tout de suite)
setInterval(poll, POLL_MS);
poll();

// Carte 2D vide en attendant les premières données
drawMap(null, null, {});

// Onglet affiché au chargement : Exploration
applyFilter('explore');
