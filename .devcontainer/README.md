# Unsere kleine Anleitung:

`crtl+shift+p`, Dev Containers:Rebuild and Reopen Container

Wenn alles fertig gebaut ist, kann man direkt die gewünschte Simulation starten:

```bash
ros2 launch vehicle_simulator system_garage.launch
```

Wenn Probleme:
Wenn vorher was kaputt war und es jetzt eig gefixed sein sollte aber trz noch nicht funktioniert, versuchs mal mit 
Dev Containers:Rebuild Without Cache and Reopen Container


History: 
das hier haben wir gemacht, sollte jetzt aber auch automatisch beim docker start schon ausgefühlt werden und nicht mehr notwendig sein

```bash
cd /workspaces/autonomous_exploration_development_environment
```

```bash
./src/vehicle_simulator/mesh/download_environments.sh
```

```bash
source /usr/share/gazebo/setup.sh
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```