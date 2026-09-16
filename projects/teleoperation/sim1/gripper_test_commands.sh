#!/usr/bin/env bash
set -e

left_open() {
  ros2 topic pub --rate 10 --times 10 /left_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [1]}"
}

left_close() {
  ros2 topic pub --rate 10 --times 10 /left_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0]}"
}

right_open() {
  ros2 topic pub --rate 10 --times 10 /right_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [1]}"
}

right_close() {
  ros2 topic pub --rate 10 --times 10 /right_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0]}"
}

both_open() {
  ros2 topic pub --rate 10 --times 10 /left_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [1]}" &
  ros2 topic pub --rate 10 --times 10 /right_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [1]}" &
  wait
}

both_close() {
  ros2 topic pub --rate 10 --times 10 /left_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0]}" &
  ros2 topic pub --rate 10 --times 10 /right_gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0]}" &
  wait
}

cycle_test() {
  while true; do
    echo "[both close]"
    both_close
    sleep 2
    echo "[both open]"
    both_open
    sleep 2
  done
}

topic_info() {
  ros2 topic info /left_gripper_controller/commands -v
  ros2 topic info /right_gripper_controller/commands -v
}

clean_fastrtps_shm() {
  sudo rm -f /dev/shm/fastrtps_port*
}

case "$1" in
  left_open) left_open ;;
  left_close) left_close ;;
  right_open) right_open ;;
  right_close) right_close ;;
  both_open) both_open ;;
  both_close) both_close ;;
  cycle) cycle_test ;;
  topic_info) topic_info ;;
  clean_shm) clean_fastrtps_shm ;;
  *)
    echo "Usage: $0 {left_open|left_close|right_open|right_close|both_open|both_close|cycle|topic_info|clean_shm}"
    ;;
esac
