# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/yxh/Documents/mars_unity/Unity-Robotics-Hub-main/tutorials/pick_and_place/ROS/devel;/home/yxh/Documents/mars_unity/Unity-Robotics-Hub-main/tutorials/pick_and_place/servo/devel;/home/yxh/Documents/ros_tutorials-melodic-devel/devel;/home/yxh/Documents/construct/devel;/home/yxh/Documents/webots/webots_ros/devel;/home/yxh/Documents/glove/devel;/home/yxh/Documents/phantom_omni/devel;/home/yxh/my_gazebo_plugin/devel;/home/yxh/Documents/ROS-Academy-for-Beginners-master/devel;/home/yxh/Documents/gazebo_ros_demos-kinetic-devel/devel;/home/yxh/Documents/rospcl/devel;/home/yxh/Documents/catkin/devel;/opt/ros/melodic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/yxh/Documents/mars_unity/Unity-Robotics-Hub-main/tutorials/pick_and_place/ROS/src/save_rgbd_from_kinect2/build/devel/env.sh')

output_filename = '/home/yxh/Documents/mars_unity/Unity-Robotics-Hub-main/tutorials/pick_and_place/ROS/src/save_rgbd_from_kinect2/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
