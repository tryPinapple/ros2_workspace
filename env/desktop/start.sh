#!/bin/bash
ENV_FILE="$(pwd)/env/desktop"

# Device list
IMU_DEVICE="/dev/ttyUSB0"
IMU_CONNECTED="--device=$IMU_DEVICE"

if ! docker images --format json | grep -q "ros2-humble"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  bash $ENV_FILE/setup.sh
fi

# Check if the device is available
if [ ! -c $IMU_DEVICE ]; then
  echo -e "[\033[0;33m WARN \033[0m] Device $IMU_DEVICE (\033[0;32mIMU\033[0m) not found."
  IMU_CONNECTED=""
fi

echo -e "[\033[0;36m INFO \033[0m] Starting ROS2 Humble desktop container..."
docker run -it --rm \
  $IMU_CONNECTED \
  --group-add dialout \
  --net=host \
  -v $(pwd)/workspace:/workspace \
  ros2-humble
