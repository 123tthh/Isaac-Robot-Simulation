cmake_minimum_required(VERSION 3.8)
project(r1_description)

find_package(ament_cmake REQUIRED)
install (
	DIRECTORY launch meshes xacro rviz urdf config
	DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/joint_state_fill.py
  scripts/check_joint_state_tf_chain.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
