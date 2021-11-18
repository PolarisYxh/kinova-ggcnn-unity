#!/usr/bin/env python

import rospy

from tcp_endpoint.RosTCPServer import TCPServer
from tcp_endpoint.RosPublisher import RosPublisher
from tcp_endpoint.RosSubscriber import RosSubscriber
from tcp_endpoint.RosService import RosService

from kinova_msgs.msg import KinovaMoveitJoints, KinovaTrajectory
from kinova_msgs.srv import MoverService
from sensor_msgs.msg import Image
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
    # #'SourceDestination_input': RosPublisher('SourceDestination', KinovaMoveitJoints, queue_size=10),
    #'NiryoTrajectory': RosSubscriber('NiryoTrajectory', KinovaTrajectory, unity_machine_ip, unity_machine_port),
    source_destination_dict = {

        '/camera/depth/image_meters': RosPublisher('/camera/depth/image_meters',Image,queue_size=10),
        'kinova_moveit': RosService('kinova_moveit', MoverService)#key:ros service in ros for plan    RosService: service    The service name in ROS   service_class:  The service node in ros
    }

    # Start the Server Endpoint
    tcp_server = TCPServer(ros_tcp_ip, ros_tcp_port, ros_node_name, source_destination_dict)# tcp server for unity
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
