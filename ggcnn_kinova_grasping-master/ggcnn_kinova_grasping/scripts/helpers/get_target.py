#! /usr/bin/env python
from __future__ import division
import time

import numpy as np

import torch

import cv2
import scipy.ndimage as ndimage
from skimage.draw import circle
from skimage.feature import peak_local_max
from models.ggcnn import GGCNN
import rospy
from helpers.cv_bridge import CvBridge,TimeIt
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
import sys



# Output publishers.

# Initialise some globals.

# Get the camera parameters
#camera_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
#K = camera_info_msg.K
# fx = K[0]
# cx = K[2]
# fy = K[4]
# cy = K[5]

# FOV_H=2*arctan(X/2f);
# FOV_V=2*arctan(Y/2f);
class get_target:
    def __init__(self) -> None:
        self.fx = 1000  #  focal_length/(sensor_size.x/pic_width)
        self.fy = 1000  #  focal_length/(sensor_size.y/pic_height)
        self.cx = 320  #  pic_width/2
        self.cy = 240  #  pic_height/2
        self.far_plane = 10.0
        self.frame_num = 0
        self.grasp_pub = rospy.Publisher('ggcnn/img/grasp', Image, queue_size=1)
        self.grasp_plain_pub = rospy.Publisher('ggcnn/img/grasp_plain', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('ggcnn/img/depth', Image, queue_size=1)
        self.ang_pub = rospy.Publisher('ggcnn/img/ang', Image, queue_size=1)
        self.cmd_pub = rospy.Publisher('ggcnn/out/command', Float32MultiArray, queue_size=1)
        self.width_pub = rospy.Publisher('ggcnn/img/width', Image, queue_size=1)
        self.prev_mp = np.array([150, 150])
        self.ROBOT_Z = 0

        self.device = torch.device("cpu")
        self.model = torch.load('./scripts/ggcnn_weights_cornell/ggcnn_epoch_23_cornell',map_location=self.device)
        self.bridge = CvBridge()
        self.pub = False
        self.get_flag = False
        self.get_normal_flag = False
        self.dir= [0,0,1]
        self.center= [0,0,0,0]
        self.pos= [0,0,0,0,0,0]

    def robot_pos_callback(self,data):
        self.ROBOT_Z = data.pose.position.z

    def numpy_to_torch(self,depth_crop):
        if len(depth_crop.shape) == 2:
            depth_crop = np.expand_dims(depth_crop, 0).astype(np.float32)
            depth_crop = torch.from_numpy(np.expand_dims(depth_crop, 0).astype(np.float32))
        else:
            depth_crop = torch.from_numpy(depth_crop.astype(np.float32))
        return depth_crop
    def save_show_normal(self,normal):
        print(normal[101,456])
        normal=(normal*0.5+0.5)*255
        normal = cv2.cvtColor(np.asarray(normal,dtype=np.float32), cv2.COLOR_BGR2RGB)
        print(normal[101,456])
        cv2.imwrite("normal1.png",normal.astype(np.uint8))
    def show_eye_img(self,msg):
        '''显示和保存真实深度图
        '''
        m=np.max(msg)
        pic=(msg)/(m)
        cv2.imshow("1",pic)
        cv2.waitKey()
        cv2.imwrite("result.png",pic*255)
    def get_normal(self):
        while 1:
            if self.get_normal_flag:
                break
        return self.dir,self.center
    def eyeDepth2normal_dir(self, depth_message):
        '''得到法线图,return -1~1
        '''
        if self.get_normal_flag:
            depth = self.bridge.imgmsg_to_cv2(depth_message)
            mask=cv2.inRange(depth,0.18,3)/255#remove far_clip
            # depth = depth*mask
            self.show_eye_img(depth)
            x = np.linspace(1,-1,3)
            w,h=depth.shape
            mask = mask[1:w-1,1:h-1]
            x = np.linspace(h/2-0.5,-h/2+0.5,h)
            y = np.linspace(w/2-0.5,-w/2+0.5,w)

            X,Y = np.meshgrid(x, y)
            X=(X)/self.fx*depth
            Y=(Y)/self.fy*depth
            depth=np.stack((X,Y,depth),axis=2)

            dy=(depth[2:w,1:h-1]-depth[0:w-2,1:h-1])*0.5
            dx=(depth[1:w-1,2:h]-depth[1:w-1,0:h-2])*0.5

            normal = np.cross(dx,dy)
            normal1=normal*normal
            n=np.sqrt(normal1[:,:,0]+normal1[:,:,1]+normal1[:,:,2])
            n=np.stack((n,n,n),axis=2)
            normal = normal/n #每个像素(x,y,z)方向法线，z正方向为指向相机平面,法线图
            # self.save_show_normal(normal)
            # 将所有法线平均，统计平面方向
            x = normal[:,:,0]*mask
            y = normal[:,:,1]*mask
            z = normal[:,:,2]*mask
            s=(normal.shape[0]*normal.shape[1])
            x=np.sum(x)/s
            y=np.sum(y)/s
            z=np.sum(z)/s
            dir = [x,y,z]
            # x=np.sum(normal,axis=1)
            # x=np.sum(x,axis=0)
            self.dir = dir/np.linalg.norm(dir)
            # self.get_normal_flag=False

    def get_point(self, depth, x,y):
        x=min(x,479)
        y=min(y,639)
        x=max(x,0)
        y=max(y,0)
        dep=depth[x][y]
        px = -(y-self.cx)/self.fx*dep
        py = -(x-self.cy)/self.fy*dep
        p = np.asarray([px,py,dep])
        return p
    def get_point_normal(self,depth,point):
        '''得到一点上的法线
        '''
        point = [int(point[0]),int(point[1])]
        up    = self.get_point(depth,point[0]+3,   point[1])
        down  = self.get_point(depth,point[0]-3,   point[1])
        left  = self.get_point(depth,point[0],     point[1]-3)
        right = self.get_point(depth,point[0],     point[1]+3)
        diry=(up-down)
        dirx=(right-left)
        dir = np.cross(dirx,diry)
        dir = dir/np.linalg.norm(dir)
        return dir
    def eyeDepth2pointnormal(self, depth_message):
        # 采样得到图片的法线,还有优化空间：inRange直接得到surface平面轮廓
        depth = self.bridge.imgmsg_to_cv2(depth_message)
        black_img = np.zeros(depth.shape,np.uint8)
        mask=cv2.inRange(depth, 0, 0.19)
        # depth = depth*mask
        # self.show_eye_img(depth)
        contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)#find handle's outline
        if len(contours)==0:
            # 优化
            # mask=cv2.inRange(depth, 0.18, 0.5)#find surface's outline
            # contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # x,y,w,h     = cv2.boundingRect(contours[0])
            # self.center = [y+h/2, x+w/2]

            self.center = [240,320]
            self.dir = self.get_point_normal(depth, self.center)

            # print("mission failed or error!")
        else:
            x,y,w,h     = cv2.boundingRect(contours[0])
            self.center = [y+h/2, x+w/2]
            center      = self.center
            # cv2.rectangle(black_img,(x,y),(x+w-1,y+h-1),(255,0,0),2)
            # cv2.imshow("2", black_img)
            # cv2.waitKey()
            # (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(depth, mask)
            dir = []
            if center[1]-w/2-20>0:
                point = [center[0], center[1]-w/2-20]
                dir.append(self.get_point_normal(depth, point))
            if center[1]+w/2+20<depth.shape[1]:
                point = [center[0],center[1]+w/2+20]
                dir.append(self.get_point_normal(depth, point))
            if center[0]-h/2-20>0:
                point = [center[0]-h/2-20,center[1]]
                dir.append(self.get_point_normal(depth, point))
            if center[0]+h/2+20<depth.shape[0]:
                point = [center[0]+h/2+20, center[1]]
                dir.append(self.get_point_normal(depth, point))
            m=-1
            idx = -1
            for i in range(0,len(dir)):
                if abs(dir[i][2])>m and abs(dir[i][2])<1:
                    m=abs(dir[i][2])
                    idx = i
            if idx==-1:
                self.dir = [0,0,1]
            else:
                self.dir = dir[idx]
        # theta = np.arctan(self.dir[0]/self.dir[2])
        # print(theta/np.pi*180)
        self.get_normal_flag = True

    def get_pos(self):
        self.get_flag = True
        while 1:
            if not self.get_flag:
                return self.pos
        return self.pos
    def depth_callback(self,depth_message):
        if self.get_flag:
            print("into ggcnn!")
            with TimeIt('Crop'):
                depth = self.bridge.imgmsg_to_cv2(depth_message)
                # mask=cv2.inRange(depth,0,0.99)/255
                # nums = np.sum(mask)
                # depth = depth*mask
                # depth[mask==0]=np.max(depth)
                # Crop a square out of the middle of the depth and resize it to 300*300
                crop_size = 400
                depth_crop = cv2.resize(depth[(480-crop_size)//2:(480-crop_size)//2+crop_size, (640-crop_size)//2:(640-crop_size)//2+crop_size], (300, 300))

                # mask = cv2.inRange(depth_crop,0,5)/255
                # depth_crop = depth_crop*mask
                # Replace nan with 0 for inpainting.
                depth_crop = depth_crop.copy()
                depth_nan = np.isnan(depth_crop).copy()
                depth_crop[depth_nan] = 0
            with TimeIt('Inpaint'):
                # open cv inpainting does weird things at the border.
                depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)

                mask = (depth_crop == 0).astype(np.uint8)
                # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
                depth_scale = np.abs(depth_crop).max()
                depth_crop = depth_crop.astype(np.float32)/depth_scale  # Has to be float32, 64 not supported.

                depth_crop = cv2.inpaint(depth_crop, mask, 1, cv2.INPAINT_NS)

                # Back to original size and value range.
                depth_crop = depth_crop[1:-1, 1:-1]
                depth_crop = depth_crop * depth_scale

            with TimeIt('Calculate Depth'):
                # Figure out roughly the depth in mm of the part between the grippers for collision avoidance.
                depth_center = depth_crop[100:141, 130:171].flatten()#change to 1 dimension
                depth_center.sort()#sort by row,small to big
                depth_center = depth_center[:10].mean() * 1000.0

            with TimeIt('Inference'):
                # Run it through the network.
                depth_crop = np.clip((depth_crop - depth_crop.mean()), -1, 1)
                depth_crop1 = self.numpy_to_torch(depth_crop)
                depth_crop_cuda = depth_crop1.to(self.device)

                pred_out = self.model(depth_crop_cuda)

                points_out = pred_out[0].squeeze().cpu().detach().numpy()#维度为1则降一维
                points_out[depth_nan] = 0

            with TimeIt('Trig'):
                # Calculate the angle map.
                cos_out = pred_out[1].squeeze().cpu().detach().numpy()
                sin_out = pred_out[2].squeeze().cpu().detach().numpy()
                ang_out = np.arctan2(sin_out, cos_out)/2.0

                width_out = pred_out[3].squeeze().cpu().detach().numpy() * 150.0  # Scaled 0-150:0-1

            with TimeIt('Filter'):
                # Filter the outputs.
                points_out = ndimage.filters.gaussian_filter(points_out, 5.0)  # cnn to filter
                ang_out = ndimage.filters.gaussian_filter(ang_out, 2.0)

            with TimeIt('Control'):
                # Calculate the best pose from the camera intrinsics.
                maxes = None

                ALWAYS_MAX = False  # Use ALWAYS_MAX = True for the open-loop solution.

                if self.ROBOT_Z > 0.34 or ALWAYS_MAX:  # > 0.34 initialises the max tracking when the robot is reset.
                    # Track the global max.
                    max_pixel = np.array(np.unravel_index(np.argmax(points_out), points_out.shape))
                    prev_mp = max_pixel.astype(np.int)
                else:
                    # Calculate a set of local maxes.  Choose the one that is closes to the previous one.
                    maxes = peak_local_max(points_out, min_distance=10, threshold_abs=0.1, num_peaks=3)
                    if maxes.shape[0] == 0:
                        self.get_flag = False
                        print("empty")
                        return
                    max_pixel = maxes[np.argmin(np.linalg.norm(maxes - self.prev_mp, axis=1))]#get the point near prev point

                    # Keep a global copy for next iteration.
                    prev_mp = (max_pixel * 0.25 + self.prev_mp * 0.75).astype(np.int)
                    max_pixel1 = max_pixel
                ang = ang_out[max_pixel[0], max_pixel[1]]
                width = width_out[max_pixel[0], max_pixel[1]]

                # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
                max_pixel = ((np.array(max_pixel) / 300.0 * crop_size) + np.array([(480 - crop_size)//2, (640 - crop_size) // 2]))
                max_pixel = np.round(max_pixel).astype(np.int)

                point_depth = depth[max_pixel[0], max_pixel[1]]
                # These magic numbers are my camera intrinsic parameters.
                x = (max_pixel[1] - self.cx)/(self.fx) * point_depth
                y = (max_pixel[0] - self.cy)/(self.fy) * point_depth
                z = point_depth
                self.pos = [x, y, z, ang, width, depth_center]
                if np.isnan(z):
                    return
            if self.pub:
                with TimeIt('Draw'):
                    # Draw grasp markers on the points_out and publish it. (for visualisation)
                    grasp_img = np.zeros((300, 300, 3), dtype=np.uint8)
                    grasp_img[:,:,2] = (points_out * 255.0)

                    grasp_img_plain = grasp_img.copy()

                    rr, cc = circle(max_pixel1[0], max_pixel1[1], 5)
                    grasp_img[rr, cc, 0] = 0
                    grasp_img[rr, cc, 1] = 255
                    grasp_img[rr, cc, 2] = 0
                    #depth[rr,cc] = 255
                    #cv2.imshow("x", depth)

                with TimeIt('Publish'):
                    # Publish the output images (not used for control, only visualisation)
                    grasp_img = self.bridge.cv2_to_imgmsg(grasp_img, 'bgr8')
                    grasp_img.header = depth_message.header
                    self.grasp_pub.publish(grasp_img)

                    grasp_img_plain = self.bridge.cv2_to_imgmsg(grasp_img_plain, 'bgr8')
                    grasp_img_plain.header = depth_message.header
                    self.grasp_plain_pub.publish(grasp_img_plain)

                    self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_crop))

                    self.ang_pub.publish(self.bridge.cv2_to_imgmsg(ang_out))
                    self.width_pub.publish(self.bridge.cv2_to_imgmsg(width_out))
                    # Output the best grasp pose relative to camera.
                    cmd_msg = Float32MultiArray()
                    cmd_msg.data = [x, y, z, ang, width, depth_center]
                    print(cmd_msg.data)
                    self.cmd_pub.publish(cmd_msg)
            else:
                self.get_flag = False

