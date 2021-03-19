cmake_minimum_required(VERSION 2.8.3)
project(kinova_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  actionlib_msgs
  geometry_msgs

  std_msgs
  tcp_endpoint
  roscpp
  rospy
  moveit_msgs
  sensor_msgs
  controller_manager
  roslaunch
  trajectory_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  FingerPosition.msg
  JointAngles.msg
  JointVelocity.msg
  JointTorque.msg
  KinovaPose.msg
  PoseVelocity.msg
  PoseVelocityWithFingers.msg
  CartesianForce.msg
  KinovaMoveitJoints.msg
  KinovaTrajectory.msg
)

add_service_files(
  FILES
  Start.srv
  Stop.srv
  HomeArm.srv
  SetForceControlParams.srv
  SetEndEffectorOffset.srv
  SetNullSpaceModeState.srv
  SetTorqueControlMode.srv
  SetTorqueControlParameters.srv
  ClearTrajectories.srv
  ZeroTorques.srv
  AddPoseToCartesianTrajectory.srv
  RunCOMParametersEstimation.srv
  MoverService.srv
)

add_action_files(DIRECTORY action
  FILES
  ArmJointAngles.action
  ArmPose.action
  Arm_KinovaPose.action
  SetFingersPosition.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs

  std_msgs
  moveit_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/mover.py
  scripts/server_endpoint.py
  scripts/server_endpoint_close.py
  scripts/trajectory_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



