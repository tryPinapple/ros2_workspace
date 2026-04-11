# Base image for ROS2 humble on jetson Xavier
FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

# 1 BULLDOZER FIX: Delete the hardcoded ROS 2 list and rebuild it with the renewed key
RUN rm -f /etc/apt/sources.list.d/ros*.list && \
    wget -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor | tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
# 2. Install standard system/pip dependencies first
RUN apt-get update && apt-get install -y python3-pip libserial-dev python3-vcstool && apt-get clean

# 3. Create a separate workspace strictly for 3rd party packages
RUN mkdir -p /opt/underlay_ws/src
WORKDIR /opt/underlay_ws

# Copy the repos file into the workspace root
COPY underlay.repos .

# Use vcs to pull all repositories into the src directory
RUN vcs import src < underlay.repos

# Ignore the test messages package
RUN touch /opt/underlay_ws/src/rosbridge_suite/rosbridge_test_msgs/COLCON_IGNORE

# Install any dependencies from rosdep, then compile
WORKDIR /opt/underlay_ws
RUN apt-get update && \
    rosdep install -i -r --from-paths src --rosdistro humble -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/humble/install/setup.bash && colcon build --cmake-args -DBUILD_TESTING=OFF

# 4. Copy the CURRENT state of source code into the image. This is only to get ROS2 package.xml & python requirements.txt files
WORKDIR /workspace
COPY ./src ./src
COPY requirements_xavier.txt /workspace/

# 5. Install python dependencies & let rosdep read the copied package.xml files and install system packages
RUN pip3 install -r requirements_xavier.txt
RUN apt-get update && \
    rosdep install -i -r --from-paths src --rosdistro humble -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 6. Automatically source ROS2 base workspaces in new terminals
RUN echo "source /opt/ros/humble/install/setup.bash" >> ~/.bashrc
RUN echo "source /opt/underlay_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# 7. Apply performance fix for control node's lqr math libraries.
# This fix is applied here because its specific to the underlying BLAS libraries
# used in this ros2 base container, which uses OpenBLAS instead of common BLAS when 
# getting scripy from apt or pypi
RUN echo "export OPENBLAS_NUM_THREADS=1" >> ~/.bashrc
RUN echo "export OMP_NUM_THREADS=1" >> ~/.bashrc
RUN echo "export MKL_NUM_THREADS=1" >> ~/.bashrc
RUN echo "export NUMEXPR_NUM_THREADS=1" >> ~/.bashrc


