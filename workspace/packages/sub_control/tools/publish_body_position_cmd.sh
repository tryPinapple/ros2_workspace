#!/bin/bash
rostopic pub /control/body_pos_target std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [$1, 0, 0]" -1
