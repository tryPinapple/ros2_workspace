#!/bin/bash
# $1: package-name

cd ./packages
ros2 pkg create --build-type ament_cmake --license Apache-2.0 $1
chgrp -R ubuntu $1
chmod -R g+w $1
cd ..