#!/usr/bin/env python

# Siemens AG, 2018
# Author: Berkay Alp Cakal (berkay_alp.cakal.ct@siemens.com)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# <http://www.apache.org/licenses/LICENSE-2.0>.
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import numpy
from sensor_msgs.msg import Joy
import geometry_msgs
from geometry_msgs.msg import TwistStamped,Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('JoyToTwist', anonymous=True)
	velo_unity_pub = rospy.Publisher('/servo_server/delta_twist_cmds',TwistStamped, queue_size=1)
	CURRENT_VELOCITY = [0,0,0,0,0,0]
	twist = geometry_msgs.msg.Twist(CURRENT_VELOCITY[0:3], CURRENT_VELOCITY[3:6])
	twist.linear = geometry_msgs.msg.Vector3(CURRENT_VELOCITY[0], CURRENT_VELOCITY[1], CURRENT_VELOCITY[2])#0, 0, 0
	twist.angular = geometry_msgs.msg.Vector3(CURRENT_VELOCITY[3], CURRENT_VELOCITY[4], CURRENT_VELOCITY[5])#0, 0, 0
	tem = geometry_msgs.msg.TwistStamped()
	tem.header.stamp = rospy.Time.now()
	tem.twist = twist
	rospy.loginfo(tem.twist)
	rospy.Duration(secs=0.01)
	velo_unity_pub.publish(tem)
	try:
		while(1):
			CURRENT_VELOCITY = [0,0,0,0,0,0]
			key = getKey()
			if key == 'w' :
				CURRENT_VELOCITY[2]=0.25
			elif key == 's':
				CURRENT_VELOCITY[2]=-0.25
			if key == 'a':
				CURRENT_VELOCITY[1]=0.25
			elif key=='d':
				CURRENT_VELOCITY[1]=-0.25
			if key=='q':
				CURRENT_VELOCITY[0]=0.25
			elif key=='e':
				CURRENT_VELOCITY[0]=-0.25
			if key == 'r' :
				CURRENT_VELOCITY[3]=0.25
			elif key == 'y':
				CURRENT_VELOCITY[3]=-0.25
			if key == 'f':
				CURRENT_VELOCITY[5]=0.25
			elif key=='h':
				CURRENT_VELOCITY[5]=-0.25
			if key=='t':
				CURRENT_VELOCITY[4]=0.25
			elif key=='g':
				CURRENT_VELOCITY[4]=-0.25
			if key=='c':
				break
			twist.linear = geometry_msgs.msg.Vector3(CURRENT_VELOCITY[0], CURRENT_VELOCITY[1], CURRENT_VELOCITY[2])#0, 0, 0
			twist.angular = geometry_msgs.msg.Vector3(CURRENT_VELOCITY[3], CURRENT_VELOCITY[4], CURRENT_VELOCITY[5])#0, 0, 0
			tem.header.stamp = rospy.Time.now()
			tem.twist = twist
			rospy.loginfo(tem.twist)
			rospy.Duration(secs=0.01)
			velo_unity_pub.publish(tem)
	except:
		print("except")

	finally:
		velo_unity_pub.publish(tem)
	# spin() simply keeps python from exiting until this node is stopped
	# rospy.spin()
	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
