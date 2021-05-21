# simu
打开ros端kinova到unity的接口：
roslaunch j2n6s300_moveit_config unity_connect_open.launch  //开环控制
roslaunch j2n6s300_moveit_config unity_connect_close.launch  //闭环控制

打开ros端niryo到unity的接口：
roslaunch niryo_moveit part_3.launch
# true
plug in realsense camera and test realsense
roslaunch realsense2_camera rs_d435_camera_with_model.launch