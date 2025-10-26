# PR_assessment
Robotics Software Engineer Technical Assessment


# 1. Allow X11 access for Docker
xhost +local:docker

# 2. Build and run
docker-compose build
docker-compose up -d

# 3. Access container and run GUI apps (like Rviz)
docker-compose exec ros_humble bash
# Inside container: rviz2

# 4. Build the Workspace
colcon build

# 5. Source the Workspace (run this in every terminal you are opening)
source install/setup.bash 

# 4. Run Task 2 (in two different terminals)
ros2 run linear_algebra_nodes serverOOP
ros2 run linear_algebra_nodes clientOOP

# 5. Before run Task 3, install urdf & xacro
apt-get update
apt-get install -y ros-humble-xacro ros-humble-urdf

# 5. Run Task 3
ros2 launch ur20_display ur20.launch.py

# 10. When done, clean up
docker-compose down
xhost -local:docker  # Optional: revert X11 permissions