#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
import argparse
import time


if __name__ == '__main__':
  try:
    rospy.init_node('move_robot_using_trajectory_msg')
    #allow gazebo to launch
    time.sleep(5)

    # Unpause the physics
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()
    prefix = "j2n6s300"

    jointpos = [0.0,2.9,1.3,4.2,1.4,0.0]
    pub = []
    for i in range(0, 6):
      topic_name = "/j2n6s300/joint_"+str(i+1)+"_position_controller/command"
      pub.append(rospy.Publisher(topic_name, Float64, queue_size=1))
    rate = rospy.Rate(100)
    count = 0
    while (count < 50):
      for i in range(0, 6):
        jointCmd = Float64()
        jointCmd.data = jointpos[i]
        pub[i].publish(jointCmd)
      count = count + 1
      #print("sleep!")
      rate.sleep()
  except rospy.ROSInterruptException:
    print "program interrupted before completion"
