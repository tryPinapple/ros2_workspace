cmake_minimum_required(VERSION 3.8)
project(sub_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclpy REQUIRED)


# 1. Install your Python module (the directory containing your __init__.py)
ament_python_install_package(${PROJECT_NAME})

# 2. Install your Python executable scripts (your ROS 2 nodes)
# This assumes your executable scripts are in a 'scripts' folder, 
# or you can point directly to the python files inside your module.
install(PROGRAMS
  #scripts/xx_node.py  TODO modify this to add node files from script directory
  DESTINATION lib/${PROJECT_NAME}
)

# 3. Install your launch files, param files, and other data
# This is where the magic happens for --symlink-install
install(DIRECTORY
  #launch uncomment if launch dir is added
  # config uncomment if config dir is added
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
