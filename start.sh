#!/bin/bash

if ! docker images --format json | grep -q "ros2-humble"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  docker build -t ros2-humble .
fi

docker run -it --rm \
  --net=host \
  -v $(pwd)/workspace:/workspace \
  ros2-humble
