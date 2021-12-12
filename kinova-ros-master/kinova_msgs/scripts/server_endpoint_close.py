#!/usr/bin/env python

import rospy

from tcp_endpoint.RosTCPServer import TCPServer
from tcp_endpoint.RosPublisher import RosPublisher
from tcp_endpoint.RosSubscriber import RosSubscriber
from tcp_endpoint.RosService import RosService

from kinova_msgs.msg import KinovaMoveitJoints, KinovaTrajectory
from kinova_msgs.srv import MoverService
from sensor_msgs.msg import Image, JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64,Float32,Float64MultiArray
def main():
    # Get variables set in rosparam used for
    # server/node communication
    ros_tcp_ip = rospy.get_param("/ROS_IP")
    ros_tcp_port = rospy.get_param("/ROS_TCP_PORT")
    print(ros_tcp_port)
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')

    unity_machine_ip = rospy.get_param("/UNITY_IP")
    unity_machine_port = rospy.get_param("/UNITY_SERVER_PORT")

    rospy.init_node(ros_node_name, anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Create ROS communication objects dictionary for routing messages
    source_destination_dict = {
        #'SourceDestination_input': RosPublisher('SourceDestination', KinovaMoveitJoints, queue_size=10),
        #'NiryoTrajectory': RosSubscriber('NiryoTrajectory', KinovaTrajectory, unity_machine_ip, unity_machine_port),
        '/camera/depth/image_meters': RosPublisher('/camera/depth/image_meters',Image,queue_size=10),
        '/camera/rgb/image_raw': RosPublisher('/camera/rgb/image_raw',Image,queue_size=10),
        '/camera/rgb1/image_raw': RosPublisher('/camera/rgb1/image_raw',Image,queue_size=10),
        #'kinova_moveit': RosService('kinova_moveit', MoverService)#key:ros service in ros for plan    RosService: service    The service name in ROS   service_class:  The service node in ros
        '/j2n6s300/joint_states': RosPublisher('/j2n6s300/joint_states',JointState, queue_size=10),
        '/clock': RosPublisher('/clock', Clock, queue_size=10),
        '/j2n6s300/joint_group_position_controller/command': RosSubscriber('/j2n6s300/joint_group_position_controller/command', Float64MultiArray, unity_machine_ip, unity_machine_port, queue_size=10),
        # '/j2n6s300/joint_1_position_controller/command': RosSubscriber('/j2n6s300/joint_1_position_controller/command', Float64, unity_machine_ip, unity_machine_port, queue_size=10),
        # '/j2n6s300/joint_2_position_controller/command': RosSubscriber('/j2n6s300/joint_2_position_controller/command', Float64, unity_machine_ip, unity_machine_port, queue_size=10),
        # '/j2n6s300/joint_3_position_controller/command': RosSubscriber('/j2n6s300/joint_3_position_controller/command', Float64, unity_machine_ip, unity_machine_port, queue_size=10),
        # '/j2n6s300/joint_4_position_controller/command': RosSubscriber('/j2n6s300/joint_4_position_controller/command', Float64, unity_machine_ip, unity_machine_port, queue_size=10),
        # '/j2n6s300/joint_5_position_controller/command': RosSubscriber('/j2n6s300/joint_5_position_controller/command', Float64, unity_machine_ip, unity_machine_port, queue_size=10),
        # '/j2n6s300/joint_6_position_controller/command': RosSubscriber('/j2n6s300/joint_6_position_controller/command', Float64, unity_machine_ip, unity_machine_port, queue_size=10),
        '/j2n6s300/fingers': RosSubscriber('/j2n6s300/fingers', Float32, unity_machine_ip, unity_machine_port, queue_size=10),
    }

    # Start the Server Endpoint
    tcp_server = TCPServer(ros_tcp_ip, ros_tcp_port, ros_node_name, source_destination_dict)# tcp server for unity
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
