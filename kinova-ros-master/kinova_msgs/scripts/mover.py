#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from kinova_msgs.srv import MoverService, MoverServiceRequest, MoverServiceResponse

robot_name = 'j2n6s300'
joint_names = [robot_name+'_joint_1', robot_name+'_joint_2', robot_name+'_joint_3', robot_name+'_joint_4', robot_name+'_joint_5', robot_name+'_joint_6']

"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles):
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.go(wait=True)

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return move_group.plan()


"""
    Creates a pick and place plan using the four states below.

    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_pick_and_place(req):
    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # TODO: Make message type a list instead
    current_robot_joint_configuration = [
        math.radians(req.joints_input.joint_00),
        math.radians(req.joints_input.joint_01),
        math.radians(req.joints_input.joint_02),
        math.radians(req.joints_input.joint_03),
        math.radians(req.joints_input.joint_04),
        math.radians(req.joints_input.joint_05),
    ]

# Pre grasp - position gripper directly above target object
#joint_trajectory:
#   header:
#     seq: 0
#     stamp:
#       secs: 0
#       nsecs:         0
#     frame_id: "world"
#   joint_names: [j2n6s300_joint_1, j2n6s300_joint_2, j2n6s300_joint_3, j2n6s300_joint_4, j2n6s300_joint_5,
#   j2n6s300_joint_6]
#   points:
#     -
#       positions: [0.0, 2.90000000056162, 1.2999999818851324, -2.069999881017797, 1.3999999497628992, 0.0]
#       velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#       accelerations: [0.8668465015920869, 0.0, 0.0, 0.0, 0.0, 0.0]
#       effort: []
#       time_from_start:
#         secs: 0
#         nsecs:         0
#     -
#       positions: [0.09532988730112738, 2.828283592720222, 1.2836072734495332, -2.178144436276084, 1.4340847044556873, 0.025063090289530316]
#       velocities: [0.2558976284109563, -0.19251107081241242, -0.04400356835261379, -0.29029652713987747, 0.09149499844998306, 0.0672778028728943]
#       accelerations: [0.27059625217286987, -0.20356880439691633, -0.0465311099301309, -0.30697100536117117, 0.09675042252977867, 0.07114220411059846]
#       effort: []
#       time_from_start:
#         secs: 0
#         nsecs: 468984444
#     -.......

    pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pre_grasp_pose)

    # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(req.pick_pose)
    pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
    grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(grasp_pose)

    # Pick Up - raise gripper back to the pre grasp position
    pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(pick_up_pose)

    # Place - move gripper to desired placement position
    place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

    previous_ending_joint_angles = place_pose.joint_trajectory.points[-1].positions
    response.trajectories.append(place_pose)

    move_group.clear_pose_targets()
    #print(response)
    return response


def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kinova_moveit_server')

    s = rospy.Service('kinova_moveit', MoverService, plan_pick_and_place)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()