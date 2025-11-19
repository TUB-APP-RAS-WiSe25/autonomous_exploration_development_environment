Unsere kleine Anleitung:
crtl+shift+p, Dev Containers:Rebuild and Reopen Container

Wenn alles fertig gebaut ist, kann man direkt die gewünschte Simulation starten:

ros2 launch vehicle_simulator system_garage.launch



History: 
das hier haben wir gemacht, sollte jetzt aber auch automatisch beim docker start schon ausgefühlt werden und nicht mehr notwendig sein

cd /workspaces/autonomous_exploration_development_environment

./src/vehicle_simulator/mesh/download_environments.sh

echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

echo "source /workspaces/autonomous_exploration_development_environment/install/setup.bash" >> ~/.bashrc

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash