cmake_minimum_required(VERSION 3.8)
project(sub_launch)

# 1. Find the basic ROS 2 build tool
find_package(ament_cmake REQUIRED)

# 2. Install the directories
# This copies the entire contents of your launch, config, and urdf folders 
# into the ROS 2 install space so the 'ros2 launch' command can find them.
install(DIRECTORY
  launch
  urdf
  #rviz add later if rviz setup added
  DESTINATION share/${PROJECT_NAME}
)

# 3. Finalize the package
ament_package()
