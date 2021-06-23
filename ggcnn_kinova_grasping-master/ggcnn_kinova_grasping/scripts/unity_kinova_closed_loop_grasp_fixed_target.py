#! /usr/bin/env python

import rospy

import numpy as np

import kinova_msgs.srv
import std_msgs.msg
import geometry_msgs.msg

import tf.transformations as tft

from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
from helpers.covariance import generate_cartesian_covariance

from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import move_to_position
import time
MAX_VELO_X = 0.25
MAX_VELO_Y = 0.15
MAX_VELO_Z = 0.085
MAX_ROTATION = 1.5
CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0]
CURRENT_FINGER_VELOCITY = [0, 0, 0]

MIN_Z = 0.01
CURR_Z = 0.35
#CURR_FORCE = 0.0
GOAL_Z = 0.0

VELO_COV = generate_cartesian_covariance(0)

GRIP_WIDTH_MM = 70
CURR_DEPTH = 350  # Depth measured from camera.

SERVO = False

finger_unity_pub = rospy.Publisher('/j2n6s300/fingers', std_msgs.msg.Float32, queue_size=1)

move2 = False #拉的动作

class Averager():
    def __init__(self, inputs, time_steps):
        self.buffer = np.zeros((time_steps, inputs))
        self.steps = time_steps
        self.curr = 0
        self.been_reset = True

    def update(self, v):
        if self.steps == 1:
            self.buffer = v
            return v
        self.buffer[self.curr, :] = v
        self.curr += 1
        if self.been_reset:
            self.been_reset = False
            while self.curr != 0:
                self.update(v)
        if self.curr >= self.steps:
            self.curr = 0
        return self.buffer.mean(axis=0)

    def evaluate(self):
        if self.steps == 1:
            return self.buffer
        return self.buffer.mean(axis=0)

    def reset(self):
        self.buffer *= 0
        self.curr = 0
        self.been_reset = True


pose_averager = Averager(6, 3)


def command_callback(target_pose, target_quaternion):
    global SERVO
    global CURR_Z, MIN_Z
    global CURR_DEPTH
    global pose_averager
    global GOAL_Z
    global GRIP_WIDTH_MM
    global VELO_COV

    #CURR_DEPTH = msg.data[5]

    if SERVO:

        # d = list(msg.data)

        # PBVS Method.

        #if d[2] > 0.150:  # Min effective range of the realsense.

            # Convert width in pixels to mm.
            # 0.07 is distance from end effector (CURR_Z) to camera.
            # 0.1 is approx degrees per pixel for the realsense.
            # if d[2] > 0.25:
            #     GRIP_WIDTH_PX = msg.data[4]
            #     GRIP_WIDTH_MM = 2 * ((CURR_Z + 0.07)) * np.tan(0.1 * GRIP_WIDTH_PX / 2.0 / 180.0 * np.pi) * 1000

            # Construct the Pose in the frame of the camera.
            # gp = geometry_msgs.msg.Pose()
            # gp.position.x = d[0]
            # gp.position.y = d[1]
            # gp.position.z = d[2]
            # q = tft.quaternion_from_euler(0, 0, -1 * d[3])
            # gp.orientation.x = q[0]
            # gp.orientation.y = q[1]
            # gp.orientation.z = q[2]
            # gp.orientation.w = q[3]

            # Calculate Pose of Grasp in Robot Base Link Frame
            # Average over a few predicted poses to help combat noise.
            # x: 0.645162225459
            # y: -0.318927784776
            # z: 0.694286052858
            # w: -0.00420092196819

            # x: 0.81874143089
            # y: -0.506091943919
            # z: 0.374129838746

            # gp_base = convert_pose(gp, 'camera_link', 'j2n6s300_link_base')
            # print(gp_base.orientation)
            # print(gp_base.position)
            #gpbo = gp_base.orientation
        e = tft.euler_from_quaternion(target_quaternion)
            # Only really care about rotation about x (e[0]). update is mean function
        av = pose_averager.update(np.array(target_pose + [e[0], e[1], e[2]]))

        # else:
        #     gp_base = geometry_msgs.msg.Pose()
        #     av = pose_averager.evaluate()

        # compute end_effector to j2n6s300_link_base's orientation and euler
        g_pose1 = geometry_msgs.msg.Pose()
        g_pose1.orientation.w = 1
        try:
            end_effector = convert_pose(g_pose1, 'j2n6s300_end_effector', 'j2n6s300_link_base')
            end_effector_list = [end_effector.orientation.x, end_effector.orientation.y, end_effector.orientation.z, end_effector.orientation.w]
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print(message)
            return
        e1 = tft.euler_from_quaternion(end_effector_list)

        # Average pick pose in base frame.
        gp_base = geometry_msgs.msg.Pose()
        gp_base.position.x = av[0]
        gp_base.position.y = av[1]
        gp_base.position.z = av[2]
        GOAL_Z = av[2]
        ang = av[3] - np.pi/2  # We don't want to align, we want to grip.
        q = tft.quaternion_from_euler(np.pi/2, 0, np.pi/2) # 绕j2n6s300_base_link's fixed x y z轴转动到正的位置;  np.pi, 0, np.pi/2 to down; np.pi/2, 0, np.pi/2 to forward
        q1 = tft.quaternion_from_euler(np.pi/2, 0, 0)#将手指竖起来
        q = tft.quaternion_multiply(q1, q)
        gp_base.orientation.x = q[0]
        gp_base.orientation.y = q[1]
        gp_base.orientation.z = q[2]
        gp_base.orientation.w = q[3]

        # Get the Position of the End Effector in the frame of the Robot base Link
        g_pose = geometry_msgs.msg.Pose()
        g_pose.position.z = 0.03  # Offset from the end_effector frame to the actual position of the fingers.
        g_pose.orientation.w = 1
        p_gripper = geometry_msgs.msg.Pose()
        try:
            p_gripper = convert_pose(g_pose, 'j2n6s300_end_effector', 'j2n6s300_link_base')
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print(message)
            return
        publish_pose_as_transform(gp_base, 'j2n6s300_link_base', 'G', 0.0)

        # Calculate Position Error. pick pose - finger pose in base_frame
        dx = (gp_base.position.x - p_gripper.position.x)
        dy = (gp_base.position.y - p_gripper.position.y)
        dz = (gp_base.position.z - p_gripper.position.z)

        # Orientation velocity control is done in the frame of the gripper,
        #  so figure out the rotation offset in the end effector frame.
        end_effector_inverse = tft.quaternion_inverse(end_effector_list)
        pgo = tft.quaternion_multiply(q, end_effector_inverse)

        q1 = [pgo[0], pgo[1], pgo[2], pgo[3]]
        # q1 = [pgo.x, pgo.y, pgo.z, pgo.w]
        e = tft.euler_from_quaternion(q1)
        dr = 1 * e[0]
        dp = 1 * e[1]
        dyaw = 1 * e[2]

        vx = max(min(dx * 2.5, MAX_VELO_X), -1.0*MAX_VELO_X)
        vy = max(min(dy * 2.5, MAX_VELO_Y), -1.0*MAX_VELO_Y)
        vz = max(min(dz - 0.04, MAX_VELO_Z), -1.0*MAX_VELO_Z)

        # Apply a nonlinearity to the velocity
        v = np.array([vx, vy, vz])
        vc = np.dot(v, VELO_COV)

        CURRENT_VELOCITY[0] = vc[0]
        CURRENT_VELOCITY[1] = vc[1]
        CURRENT_VELOCITY[2] = vc[2]

        CURRENT_VELOCITY[3] = 1 * dr #x: end effector self rotate
        CURRENT_VELOCITY[4] = 1 * dp #y: up and down rotate
        CURRENT_VELOCITY[5] = 1 * dyaw #max(min(1 * dyaw, MAX_ROTATION), -1 * MAX_ROTATION) #z: left and right rotate


# def robot_wrench_callback(msg):
#     # Monitor force on the end effector, with some smoothing.
#     global CURR_FORCE
#     CURR_FORCE = 0.5 * msg.wrench.force.z + 0.5 * CURR_FORCE


def finger_position_callback(msg):
    global SERVO
    global CURRENT_FINGER_VELOCITY
    global CURR_DEPTH
    global CURR_Z
    global GRIP_WIDTH_MM

    # Only move the fingers when we're 200mm from the table and servoing.
    if CURR_Z < 0.200 and CURR_DEPTH > 80 and SERVO:
        # 4000 ~= 70mm
        g = min((1 - (min(GRIP_WIDTH_MM, 70)/70)) * (6800-4000) + 4000, 5500)

        # Move fast from fully open.
        gain = 2
        if CURR_Z > 0.12:
            gain = 5

        err = gain * (g - msg.finger1)
        CURRENT_FINGER_VELOCITY = [err, err, 0]

    else:
        CURRENT_FINGER_VELOCITY = [0, 0, 0]


def robot_position_callback(msg):#只要 z 坐标
    global SERVO
    global CURR_Z
    global CURR_DEPTH
    #global CURR_FORCE
    global VELO_COV
    global pose_averager
    global start_record_srv
    global stop_record_srv

    CURR_X = msg.pose.position.x

    # Stop Conditions.
    if CURR_Z < MIN_Z or (CURR_Z - 0.01) < GOAL_Z: #or CURR_FORCE < -5.0:
        if SERVO:
            SERVO = False

            # Grip.
            rospy.sleep(0.1)
            set_finger_positions([8000, 8000])
            rospy.sleep(0.5)

            # Move Home.
            move_to_position([0, -0.38, 0.35], [0.99, 0, 0, np.sqrt(1-0.99**2)])
            rospy.sleep(0.25)

            # stop_record_srv(std_srvs.srv.TriggerRequest())

            raw_input('Press Enter to Complete')

            # Generate a control nonlinearity for this run.
            VELO_COV = generate_cartesian_covariance(0.0)

            # Open Fingers
            set_finger_positions([0, 0])
            rospy.sleep(1.0)

            pose_averager.reset()

            raw_input('Press Enter to Start')

            # start_record_srv(std_srvs.srv.TriggerRequest())
            rospy.sleep(0.5)
            SERVO = True


if __name__ == '__main__':
    rospy.init_node('kinova_velocity_control')

    position_sub = rospy.Subscriber('/j2n6s300_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, robot_position_callback, queue_size=1)# 只要z坐标
    finger_sub = rospy.Subscriber('/j2n6s300_driver/out/finger_position', kinova_msgs.msg.FingerPosition, finger_position_callback, queue_size=1)# 只要
    #wrench_sub = rospy.Subscriber('/j2n6s300_driver/out/tool_wrench', geometry_msgs.msg.WrenchStamped, robot_wrench_callback, queue_size=1)
    command_sub = rospy.Subscriber('/ggcnn/out/command', std_msgs.msg.Float32MultiArray, command_callback, queue_size=1)

    # https://github.com/dougsm/rosbag_recording_services
    # start_record_srv = rospy.ServiceProxy('/data_recording/start_recording', std_srvs.srv.Trigger)
    # stop_record_srv = rospy.ServiceProxy('/data_recording/stop_recording', std_srvs.srv.Trigger)

    #start_force_srv = rospy.ServiceProxy('/j2n6s300_driver/in/start_force_control', kinova_msgs.srv.Start)
    #start_force_srv.call(kinova_msgs.srv.StartRequest())

    # Publish velocity at 100Hz.
    velo_pub = rospy.Publisher('/j2n6s300_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=1)
    velo_unity_pub = rospy.Publisher('/servo_server/delta_twist_cmds', geometry_msgs.msg.TwistStamped, queue_size=1)
    finger_pub = rospy.Publisher('/j2n6s300_driver/in/finger_velocity', kinova_msgs.msg.FingerPosition, queue_size=1)
    r = rospy.Rate(100)

    #move_to_position([0, -0.38, 0.35], [0.99, 0, 0, np.sqrt(1-0.99**2)])
    #rospy.sleep(0.5)
    #set_finger_positions([0, 0])
    #rospy.sleep(0.5)

    SERVO = True

    while not rospy.is_shutdown():
        if SERVO:
            if move2 == False:
                command_callback([0.61874143089, -0.506091943919, 0.376129838746], [0.645162225459, -0.318927784776, 0.694286052858, -0.00420092196819] )
                reach = True
                for tem in CURRENT_VELOCITY:
                    if(abs(tem)>1e-2):
                        reach = False
                #control fingers
                x = std_msgs.msg.Float32()
                if reach:
                    x.data = 0   # 0为close, 1为open
                    finger_unity_pub.publish(x)
                    move2 = True
                    time.sleep(2)
                    print('reach')
                else:
                    x.data = 1
                    finger_unity_pub.publish(x)
            elif move2 == True:
                command_callback([0.21874143089, -0.506091943919, 0.376129838746], [0.645162225459, -0.318927784776, 0.694286052858, -0.00420092196819] )
                reach = True
                for tem in CURRENT_VELOCITY:
                    if(abs(tem)>1e-2):
                        reach = False
                #control fingers
                x = std_msgs.msg.Float32()
                if reach:
                    x.data = 1   # 0为close, 1为open
                    finger_unity_pub.publish(x)
                    move2 = True
                else:
                    x.data = 0
                    finger_unity_pub.publish(x)
                    print('reach1')
            #finger_pub.publish(kinova_msgs.msg.FingerPosition(*CURRENT_FINGER_VELOCITY))
            velo_pub.publish(kinova_msgs.msg.PoseVelocity(*CURRENT_VELOCITY))
            twist = geometry_msgs.msg.Twist(CURRENT_VELOCITY[0:3], CURRENT_VELOCITY[3:6])
            twist.linear = geometry_msgs.msg.Vector3(CURRENT_VELOCITY[0], CURRENT_VELOCITY[1], CURRENT_VELOCITY[2])#0, 0, 0
            twist.angular = geometry_msgs.msg.Vector3(CURRENT_VELOCITY[3], CURRENT_VELOCITY[4], CURRENT_VELOCITY[5])#0, 0, 0
            tem = geometry_msgs.msg.TwistStamped()
            tem.twist = twist
            #rospy.loginfo(tem.twist)
            velo_unity_pub.publish(tem)

            CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0]
        #r.sleep()
        #rospy.spin()
