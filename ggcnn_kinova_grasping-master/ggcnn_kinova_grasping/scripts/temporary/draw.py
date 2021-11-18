from re import S
import cv2
import numpy as np
# import mxnet as mx
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# 求法线 法线图

fx = 1000  #  focal_length/(sensor_size.x/pic_width)
fy = 1000  #  focal_length/(sensor_size.y/pic_height)
cx = 320  #  pic_width/2
cy = 240  #  pic_height/2
grasp_pixel = [100,455]
def show_eye_img(msg):
    '''显示和保存真实深度图
    '''
    m=np.max(msg)
    pic=(msg)/(m)
    cv2.imshow("1",pic)
    cv2.waitKey()
    cv2.imwrite("result.png",pic*255)
# def depth2normal(depth):
#     '''depth is 0-255,is not precise
#     '''
#     # dz/dx=(z(x+1,y)-z(x-1,y))/2.0;
#     # dz/dy=(z(x,y+1)-z(x,y-1))/2.0;
#     # direction=(-dz/dx,-dz/dy,1.0)
#     # magnitude=sqrt(direction.x**2 + direction.y**2 + direction.z**2)
#     # normal=direction/magnitude
#     w,h=depth.shape
#     dx=-(depth[2:w,1:h-1]-depth[0:w-2,1:h-1])*0.5
#     dy=-(depth[1:w-1,2:h]-depth[1:w-1,0:h-2])*0.5
#     dz=mx.nd.ones((w-2,h-2))
#     dl = mx.nd.sqrt(mx.nd.elemwise_mul(dx, dx) + mx.nd.elemwise_mul(dy, dy) + mx.nd.elemwise_mul(dz, dz))
#     dx = mx.nd.elemwise_div(dx, dl) * 0.5 + 0.5
#     dy = mx.nd.elemwise_div(dy, dl) * 0.5 + 0.5
#     dz = mx.nd.elemwise_div(dz, dl) * 0.5 + 0.5
#     return np.concatenate([dy.asnumpy()[np.newaxis,:,:],dx.asnumpy()[np.newaxis,:,:],dz.asnumpy()[np.newaxis,:,:]],axis=0)
def save_show_normal(normal):
    print(normal[101,456])
    normal=(normal*0.5+0.5)*255
    normal = cv2.cvtColor(np.asarray(normal,dtype=np.float32), cv2.COLOR_BGR2RGB)
    print(normal[101,456])
    cv2.imwrite("normal1.png",normal.astype(np.uint8))
def eyeDepth2normal(depth):
    '''得到法线图,return -1~1
    '''
    grasp_point = depth[grasp_pixel[0],grasp_pixel[1]]
    x = np.linspace(1,-1,3)
    w,h=depth.shape
    x = np.linspace(h/2-0.5,-h/2+0.5,h)
    y = np.linspace(w/2-0.5,-w/2+0.5,w)

    X,Y = np.meshgrid(x, y)
    X=(X)/fx*depth
    Y=(Y)/fy*depth
    depth=np.stack((X,Y,depth),axis=2)

    dy=(depth[2:w,1:h-1]-depth[0:w-2,1:h-1])*0.5
    dx=(depth[1:w-1,2:h]-depth[1:w-1,0:h-2])*0.5

    normal = np.cross(dx,dy)
    normal1=normal*normal
    n=np.sqrt(normal1[:,:,0]+normal1[:,:,1]+normal1[:,:,2])
    n=np.stack((n,n,n),axis=2)
    normal = normal/n #每个像素(x,y,z)方向法线，z正方向为指向相机平面,法线图
    return normal
def eyeDepth2normal_dir(depth):
    '''得到法线图,return -1~1
    '''
    mask=cv2.inRange(depth,0,9.99)/255

    grasp_point = depth[grasp_pixel[0],grasp_pixel[1]]
    x = np.linspace(1,-1,3)
    w,h=depth.shape
    mask = mask[1:w-1,1:h-1]
    x = np.linspace(h/2-0.5,-h/2+0.5,h)
    y = np.linspace(w/2-0.5,-w/2+0.5,w)

    X,Y = np.meshgrid(x, y)
    X=(X)/fx*depth
    Y=(Y)/fy*depth
    depth=np.stack((X,Y,depth),axis=2)

    dy=(depth[2:w,1:h-1]-depth[0:w-2,1:h-1])*0.5
    dx=(depth[1:w-1,2:h]-depth[1:w-1,0:h-2])*0.5

    normal = np.cross(dx,dy)
    normal1=normal*normal
    n=np.sqrt(normal1[:,:,0]+normal1[:,:,1]+normal1[:,:,2])
    n=np.stack((n,n,n),axis=2)
    normal = normal/n #每个像素(x,y,z)方向法线，z正方向为指向相机平面,法线图
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
    dir = dir/np.linalg.norm(dir)
    print(normal[100,455])
    return dir
def get_point(depth, x,y):
    dep=depth[x,y]
    px = -(y-cx)/fx*dep
    py = -(x-cy)/fy*dep
    p = np.asarray([px,py,dep])
    return p
def get_point_normal(depth,point):
    '''得到一点上的法线
    '''
    up    = get_point(depth,point[0]+1,     point[1])
    down  = get_point(depth,point[0]-1,   point[1])
    left  = get_point(depth,point[0], point[1]-1)
    right = get_point(depth,point[0],point[1]+1)
    diry=(up-down)*0.5
    dirx=(right-left)*0.5
    dir = np.cross(dirx,diry)
    dir = dir/np.linalg.norm(dir)
    return dir
def eyeDepth2meannormal(depth):
    #采样得到图片的法线
    # mask=cv2.inRange(depth,0,9.99)
    # (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(depth, mask)
    point = [int(depth.shape[0]/4),int(depth.shape[1]/4)]
    dir = []
    dir.append(get_point_normal(depth, point))
    point = [int(depth.shape[0]/4),int(depth.shape[1]/4*3)]
    dir.append(get_point_normal(depth, point))
    point = [int(depth.shape[0]/4*3),int(depth.shape[1]/4*3)]
    dir.append(get_point_normal(depth, point))
    point = [int(depth.shape[0]/4*3),int(depth.shape[1]/4)]
    dir.append(get_point_normal(depth, point))
    m=1
    idx = -1
    for i in range(0,len(dir)-1):
        if abs(dir[i][2])<m:
            m=abs(dir[i][2])
            idx = i
    dir = dir[idx]
    print(dir)
def eyeDepth2normal_dir( depth_message):
    '''得到法线图,return -1~1
    '''
    bridge = CvBridge()
    depth = bridge.imgmsg_to_cv2(depth_message)
    show_eye_img(depth)
if __name__ == '__main__':
    depth=cv2.imread("scripts/temporary/depth_9.exr",cv2.IMREAD_UNCHANGED)
    # depth_sub = rospy.Subscriber('/camera/depth/image_meters', Image, eyeDepth2normal_dir, queue_size=1)
    depth = depth[:,:,2]

    normal = eyeDepth2meannormal(depth)
    # save_show_normal(normal)
    # eyeDepth2meannormal(depth)
    while not rospy.is_shutdown():
        pass
