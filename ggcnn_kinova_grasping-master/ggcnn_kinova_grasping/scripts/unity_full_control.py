#! /usr/bin/env python

from re import T
import rospy

import numpy as np

import kinova_msgs.srv
import std_msgs.msg
import geometry_msgs.msg
import helpers.transformatins as tf1
import tf.transformations as tft

from helpers.transforms import current_robot_pose, publish_tf_quaterion_as_transform, convert_pose, publish_pose_as_transform
from helpers.covariance import generate_cartesian_covariance

from helpers.gripper_action_client import set_finger_positions
from helpers.position_action_client import move_to_position
from sensor_msgs.msg import Image, CameraInfo
from helpers.get_target import get_target
import time
MAX_VELO_X = 0.1
MAX_VELO_Y = 0.15
MAX_VELO_Z = 0.15
MAX_VELO = 0.25
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

move2 = 0 #拉的动作


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

def get_target_3d(d):
    gp = geometry_msgs.msg.Pose()
    gp.position.x = -d[0]
    gp.position.y = -d[1]
    gp.position.z = d[2]
    gp.orientation.w = 1
    q = tft.quaternion_from_euler(d[3], d[4], d[5])
    gp.orientation.x = q[0]
    gp.orientation.y = q[1]
    gp.orientation.z = q[2]
    gp.orientation.w = q[3]
    # Convert to base frame, add the angle in (ensures planar grasp, camera isn't guaranteed to be perpendicular).
    gp_base = convert_pose(gp, 'camera_link', 'j2n6s300_link_base')
    if gp_base==None:
        av = np.array([0,0,0,0,0,0,0])
        return av
    gpbo = gp_base.orientation
    # e = tft.euler_from_quaternion([gpbo.x, gpbo.y, gpbo.z, gpbo.w])
    # Only really care about rotation about x (e[0]). update is mean function
    # av = pose_averager.update(np.array([gp_base.position.x, gp_base.position.y, gp_base.position.z, e[0],e[1],e[2]]))
    av = np.array([gp_base.position.x, gp_base.position.y, gp_base.position.z, gpbo.x, gpbo.y, gpbo.z, gpbo.w])
    return av

def open_servo(av):
    global SERVO
    global CURR_Z, MIN_Z
    global CURR_DEPTH
    global pose_averager
    global GOAL_Z
    global GRIP_WIDTH_MM
    global VELO_COV
    #CURR_DEPTH = msg.data[5]

    if SERVO:
        # Average pick pose in base frame.
        gp_base = geometry_msgs.msg.Pose()
        gp_base.position.x = av[0]
        gp_base.position.y = av[1]
        gp_base.position.z = av[2]
        gp_base.orientation.x = av[3]
        gp_base.orientation.y = av[4]
        gp_base.orientation.z = av[5]
        gp_base.orientation.w = av[6]
        # publish_pose_as_transform(gp_base, 'j2n6s300_link_base', 'G4', 1.0)
        # Get the Position of the End Effector in the frame of the Robot base Link
        g_pose = geometry_msgs.msg.Pose()
        g_pose.position.z = 0.05  # Offset from the end_effector frame to the actual position of the fingers.
        g_pose.orientation.w = 1
        p_gripper = geometry_msgs.msg.Pose()
        try:
            p_gripper = convert_pose(g_pose, 'j2n6s300_end_effector', 'j2n6s300_link_base')
            end_effector_list = [p_gripper.orientation.x, p_gripper.orientation.y, p_gripper.orientation.z, p_gripper.orientation.w]
            # publish_pose_as_transform(p_gripper, 'j2n6s300_link_base', 'G', 1)
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print(message)
            return

        # Calculate Position Error. pick pose - finger pose in base_frame
        dx = (gp_base.position.x - p_gripper.position.x)    #forward
        dy = (gp_base.position.y - p_gripper.position.y)    #left
        dz = (gp_base.position.z - p_gripper.position.z)    #up

        # Orientation velocity control is done in the frame of the gripper,
        #  so figure out the rotation offset in the end effector frame.
        end_effector_inverse = tft.quaternion_inverse(end_effector_list)
        pgo = tft.quaternion_multiply(av[3:], end_effector_inverse)

        q1 = [pgo[0], pgo[1], pgo[2], pgo[3]]
        e = tft.euler_from_quaternion(q1)

        if(np.sqrt(dx*dx+dy*dy+dz*dz)<0.001):
            vx=0
            vy=0
            vz=0
            dm = max(max(abs(e[0]),abs(e[1])),abs(e[2]))
            if dm>0.001:
                dr = e[0]/dm*MAX_VELO
                dp = e[1]/dm*MAX_VELO
                dyaw = e[2]/dm*MAX_VELO
            else:
                return
        else:
            dm = max(max(abs(dx),abs(dy)),abs(dz))
            vx = dx/dm*MAX_VELO
            vy = dy/dm*MAX_VELO
            vz = dz/dm*MAX_VELO
            t  = dm/MAX_VELO

            dr = 1 * e[0]/t
            dp = 1 * e[1]/t
            dyaw = 1 * e[2]/t
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

    velo_unity_pub = rospy.Publisher('/servo_server/delta_twist_cmds', geometry_msgs.msg.TwistStamped, queue_size=1)
    finger_pub = rospy.Publisher('/j2n6s300_driver/in/finger_velocity', kinova_msgs.msg.FingerPosition, queue_size=1)
    r = rospy.Rate(100)

    SERVO = True
    ggcnn=get_target()
    cam2finger_center=np.array([0, 0.1, 0.187, 0, 0, np.pi])
    move2 = 1
    close_finger = 1

    depth_sub=None
    if move2 == 0:
        ggcnn.pub = False
        depth_sub = rospy.Subscriber('/camera/depth/image_meters', Image, ggcnn.depth_callback, queue_size=1)
        d = ggcnn.get_pos() # x positive:down; y positive:left; z positive:forward; ang x:rotate up, ang y positive:rotate right,ang z positive: rotate
        d = [-0.05, 0.45, 0.38, 0, 0, d[3]+np.pi-0.1749329]
        # d = [0, 0, 0.55, 0, 0, np.pi-0.1749329]# for partnet_1bee7b073a38d30a4c3aee8e9d3a6ffa link_0
        av=get_target_3d(d)
        # depth_sub.unregister()
    while not rospy.is_shutdown():
        if SERVO:
            if move2 == 0:#reach handle
                # d = ggcnn.get_pos()
                # d = [d[0], d[1], d[2], 0, 0, d[3]+np.pi-0.1749329]
                # # d = [0, 0, 0.55, 0, 0, np.pi-0.1749329]
                # av=get_target_3d(d)
                open_servo(av)
                if CURRENT_VELOCITY==[0,0,0,0,0,0]:
                    #control fingers
                    x = std_msgs.msg.Float32()
                    x.data = 0   # 0为close, 1为open
                    finger_unity_pub.publish(x)

                    time.sleep(2)
                    print('reach')
                    move2 = 1
                    close_finger=True
                # else:
                #     x.data = 1
                #     finger_unity_pub.publish(x)
            elif move2 == 1:    # draw
                if close_finger:
                    #control fingers
                    x = std_msgs.msg.Float32()
                    x.data = 0   # 0为close, 1为open
                    finger_unity_pub.publish(x)
                    # get cam2finger_center relative pos
                    # g_pose = geometry_msgs.msg.Pose()
                    # g_pose.orientation.w = 1
                    # relative_pose = convert_pose(g_pose,'camera_link','j2n6s300_end_effector')
                    # cam2finger_center = np.array([0, -relative_pose.position.y, -relative_pose.position.z+0.05, 0, 0, np.pi]) #if d=cam2finger_center,has no movement
                    time.sleep(2)
                    print('reach')
                    depth_sub = rospy.Subscriber('/camera/depth/image_meters', Image, ggcnn.eyeDepth2pointnormal, queue_size=1)
                    close_finger = False


                # d = np.array([0, 0, -0.02, 0, -0.303687636, 0])
                dir = ggcnn.get_normal()

                if abs(dir[0])>abs(dir[1]):
                    theta = np.arctan(dir[0]/dir[2])
                    print(theta/np.pi*180)
                    d = np.array([0, 0, -0.01, 0, -theta, 0])
                else:
                    theta = np.arctan(dir[1]/dir[2])
                    d = np.array([0, 0, -0.01, -theta, 0, 0])
                d = d+cam2finger_center
                av=get_target_3d(d)
                open_servo(av)
                #control fingers
                # x = std_msgs.msg.Float32()
                # if reach:
                #     x.data = 1   # 0为close, 1为open
                #     finger_unity_pub.publish(x)
                #     move2 = 1
                # else:
                #     x.data = 0
                #     finger_unity_pub.publish(x)
                #     print('reach1')

            twist = geometry_msgs.msg.Twist(CURRENT_VELOCITY[0:3], CURRENT_VELOCITY[3:6])
            twist.linear = geometry_msgs.msg.Vector3(CURRENT_VELOCITY[0], CURRENT_VELOCITY[1], CURRENT_VELOCITY[2])#0, 0, 0
            twist.angular = geometry_msgs.msg.Vector3(CURRENT_VELOCITY[3], CURRENT_VELOCITY[4], CURRENT_VELOCITY[5])#0, 0, 0
            tem = geometry_msgs.msg.TwistStamped()
            tem.twist = twist
            rospy.loginfo(tem.twist)
            velo_unity_pub.publish(tem)

            CURRENT_VELOCITY = [0, 0, 0, 0, 0, 0]