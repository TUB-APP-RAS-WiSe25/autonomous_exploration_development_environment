#!/bin/bash
set -e

# Zur Sicherheit in den Workspace wechseln
cd /workspaces/autonomous_exploration_development_environment

echo "--- Starte Environment Setup ---"

MESH_DIR="./src/vehicle_simulator/mesh"

# 1. Check: Existieren die Ordner schon?
# Das Skript lädt normalerweise Ordner wie "garage" oder "forest" herunter.
# Wenn der Ordner "garage" da ist, gehen wir davon aus, dass der Download schon lief.
if [ -d "$MESH_DIR/garage" ]; then
    echo "Umgebungs-Dateien (z.B. garage) bereits gefunden. Überspringe Download."
else
    if [ -f "$MESH_DIR/download_environments.sh" ]; then
        echo "Lade Environments herunter..."
        bash "$MESH_DIR/download_environments.sh"
    else
        echo "WARNUNG: Download-Script nicht gefunden unter $MESH_DIR"
    fi
fi

# 2. .bashrc Einträge (nur hinzufügen, wenn noch nicht da)
if ! grep -q "source /usr/share/gazebo/setup.sh" ~/.bashrc; then
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
fi

if ! grep -q "source /workspaces/autonomous_exploration_development_environment/install/setup.bash" ~/.bashrc; then
    echo "source /workspaces/autonomous_exploration_development_environment/install/setup.bash" >> ~/.bashrc
fi

# 3. Build Prozess
source /opt/ros/humble/setup.bash
# Falls Gazebo da ist, sourcen
[ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh

echo "Starte Colcon Build (Release)..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "--- Setup fertig! ---"