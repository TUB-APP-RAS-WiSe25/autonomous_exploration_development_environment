Zum Docker auf Roboter starten:

In den root Ordner gehen wo die Dockerfile liegt, dann sollte auch alles aus diesem Ordner im Container landen:

docker build -t ws2025 .

docker run -it --rm \
    --net=host \
    --runtime=nvidia \
    -v $(pwd):/workspaces/autonomous_exploration_development_environment \
    -w /workspaces/autonomous_exploration_development_environment \
    ws2025




Anschließend im Container:

sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

Damit wir keine Probleme mit den Gazebo bekommen:

touch src/vehicle_simulator/COLCON_IGNORE
touch src/velodyne_simulator/COLCON_IGNORE

cd src
ros2 pkg create --build-type ament_cmake real_robot_bringup
mkdir -p real_robot_bringup/launch

Dann in src/real_robot_bringup/CMakeLists.txt und vor ament_package() das eintragen (zb mit sudo apt install vim):

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

vim real_robot_bringup/launch/start_robot.launch.py 
und die launch file da reinschreiben

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

ros2 launch real_robot_bringup start_robot.launch.py


HINWEIS: es gibt in vehicle_simulator/launch auch eine system_real_robot.launch, die kann man auch in unseren real_robot_bringup ordner kopieren und mit ros2 launch real_robot_bringup system_real_robot.launch starten. Bisher funktionieren beide nicht.