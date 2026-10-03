#!/bin/bash
# Installation, étape 2/5 : ajoute le dépôt apt officiel d'OSRF et installe le simulateur
# Gazebo Harmonic (paquet gz-harmonic), puis vérifie que `gz sim` répond.
# Gazebo sert à la première phase du projet (scripts/06_launch_multi_drones.sh).
#   bash scripts/installation/02_install_gazebo.sh
# Arrête le script à la première commande en échec
set -e

echo "=============================================="
echo " [2/5] Installation de Gazebo Harmonic"
echo "=============================================="

# --- Ajouter le dépôt OSRF Gazebo ---
sudo apt-get install -y lsb-release gnupg

# Clé GPG
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Dépôt
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update

# --- Installer Gazebo Harmonic ---
sudo apt-get install -y gz-harmonic

echo ""
echo "==> Vérification de l'installation de Gazebo :"
gz sim --version 2>/dev/null && echo "Gazebo Harmonic installé avec succès !" || echo "ATTENTION : gz sim non trouvé, vérifier l'installation."

