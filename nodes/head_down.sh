#!/bin/bash

# Wait for the command topic to be there
until rostopic info /pepper/Head_controller/command > /dev/null 2>&1
do
  echo "Waiting for controllers to be ready..."
  sleep 1.0
done

echo "Lowering Head."
sleep 5.0

rostopic pub --once /pepper/Head_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: [HeadPitch, HeadYaw]
points:
- positions: [0.1, -0.0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}" &


wait
