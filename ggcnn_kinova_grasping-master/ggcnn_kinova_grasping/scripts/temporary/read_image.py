# 读取unity depth camera模块 depth01.mat输出的exr格式深度图并生成点云数据
import cv2
from numpy.core.function_base import linspace
import scipy.ndimage as ndimage
from skimage.draw import circle,line
import numpy as np

# import pcl.pcl_visualization
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
def show_pcl1(msg):
    # lidar_path 指定一个kitti 数据的点云bin文件就行了
    points = msg.reshape(-1, 4)  # .astype(np.float16)
    cloud = pcl.PointCloud(points[:,:3])
    visual = pcl.pcl_visualization.CloudViewing()
    visual.ShowMonochromeCloud(cloud)
    flag = True
    while flag:
        flag != visual.WasStopped()

def show_pcl_img(msg):
    '''
    msg is x*y*4
    '''
    x=msg[:,:,0].flatten()
    y=msg[:,:,1].flatten()
    z=msg[:,:,2].flatten()
    #开始绘图
    fig=plt.figure(dpi=120)
    ax=fig.add_subplot(111,projection='3d')
    #标题
    plt.title('point cloud')
    #利用xyz的值，生成每个点的相应坐标（x,y,z）
    ax.scatter(x,y,z,c='b',marker='.',s=2,linewidth=0,alpha=1,cmap='spectral')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    #显示
    plt.show()
def show_pcl(msg):
    '''
    msg is 4*...
    '''
    x=msg[0]
    y=msg[1]
    z=msg[2]
    #开始绘图
    fig=plt.figure(dpi=120)
    ax=fig.add_subplot(111,projection='3d')
    #标题
    plt.title('point cloud')
    #利用xyz的值，生成每个点的相应坐标（x,y,z）
    ax.scatter(x,y,z,c='b',marker='.',s=2,linewidth=0,alpha=1,cmap='spectral')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    #显示
    plt.show()
def show_eye_img(msg):
    '''显示和保存真实深度图
    '''
    m=np.max(msg)
    pic=(msg)/(m)
    cv2.imshow("1",pic)
    cv2.waitKey()
    cv2.imwrite("result.png",pic*255)
def pointMat2txt(img_msg):
    # read unity depth camera模块 point.mat输出的exr格式点云图并转换
    img_msg1=cv2.imread("scripts/pcl_48.exr",cv2.IMREAD_UNCHANGED)
    img_msg1=cv2.cvtColor(img_msg1,cv2.COLOR_RGBA2BGRA)
    print(img_msg1)
    img_msg1=img_msg1.reshape(480*640,-1)
    img_msg1=np.flipud(img_msg1)

    img_msg1=img_msg1[:,:,0:3]
    img_msg1=img_msg1.reshape(480*640,-1)
    np.savetxt("b.txt", img_msg1,fmt='%.15f')
def zvalue2pcl(img_msg):
    img_msg1=cv2.imread("scripts/pcl_48.exr",cv2.IMREAD_UNCHANGED)
    # 读取unity depth camera模块 depth01.mat输出的exr格式点云图并转换
    img_msg=img_msg[:,:,2]
    show_eye_img(img_msg)
    img_msg=np.flipud(img_msg)
    img_msg=img_msg*2-1
    print(img_msg)
    width=img_msg.shape[1]
    height=img_msg.shape[0]
    x = np.linspace(-(1-1/width),(1-1/width),640)
    y = np.linspace(-(1-1/height),(1-1/height),480)
    X,Y = np.meshgrid(x, y)
    w=np.ones(img_msg.shape)
    point=np.stack((X,Y,img_msg,w),axis=2)
    print(point)
    point=np.transpose(point)
    print(point)
    point=point.reshape(4,-1)
    projection_inverse=[[0.32000,	0.00000,	0.00000,	0.00000],
                        [0.00000,	0.19692,	0.00000,	0.00000],
                        [0.00000,	0.00000,	0.00000,	-1.00000],
                        [0.00000,	0.00000,	-4.95000,	5.05000]]
    # VP_inverse=[[-0.27711,	-0.03719,	-0.84077,	1.32082],
    # [0.06309,	-0.19296,	-4.52577,	4.58625],
    # [0.14707,	0.01270,	-0.37349,	1.26682],
    # [0.00000,	0.00000,	-4.95000,	5.05000]]
    pcl=np.matmul(projection_inverse,point)
    w=pcl[3]
    pcl=pcl/w
    print(pcl)
    pcl=np.transpose(pcl)
    np.set_printoptions(suppress=True)
    np.savetxt("a.txt", pcl,fmt='%.15f')

def draw_bbox(rgb,x,y,width,height):
    '''在图上画线和画点来显示
    '''
    # "x": 327.0,
    # "y": 128.0,
    # "width": 28.0,
    # "height": 15.0
    rr,cc=line(y,x,y,x+width)
    rgb[rr,cc]=[255,255,255,255]
    rgb[rr-1,cc]=[255,255,255,255]#加粗
    rr,cc=line(y,x,y+height,x)
    rgb[rr,cc]=[255,255,255,255]
    rgb[rr,cc-1]=[255,255,255,255]
    rr,cc=line(y+height,x+width,y+height,x)
    rgb[rr,cc]=[255,255,255,255]
    rgb[rr+1,cc]=[255,255,255,255]
    rr,cc=line(y+height,x+width,y,x+width)
    rgb[rr,cc]=[255,255,255,255]
    rgb[rr,cc+1]=[255,255,255,255]
    cv2.imshow("1",rgb)
    cv2.waitKey()
    cv2.imwrite("result.png",rgb)
def show_contour(black_img,rect,contours):
    points=cv2.boxPoints(rect)

    for j in range(0,4):
        x=(int(points[j][0]),int(points[j][1]))
        y=(int(points[(j+1)%4][0]),int(points[(j+1)%4][1]))
        cv2.line( black_img, x, y, (0,0,255), 1, 8 )
    # cv2.drawContours(black_img, contours, -1, (0,0,255), 3)
    cv2.imshow("1",black_img)
    cv2.waitKey()
def get_obb(rgb):
    '''get_obb from segmentation
    movableMask green
        handle blue

        "x": 268.0,
        "y": 197.0,
        "width": 3.0,
        "height": 6.0
    '''
    low=np.asarray([255,0,0])
    up=np.asarray([255,0,0])
    mask=cv2.inRange(rgb,low,up)
    black_img = np.zeros(rgb.shape,np.uint8)
    # ret,threshold_output=cv2.threshold( rgb, 250, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    rect = cv2.minAreaRect(contours[0])
    # rect    = cv2.boundingRect(contours[0])
    if rect[1][0]>rect[1][1]:
        ang = 180-rect[2]
        width = rect[1][1]
    else:
        width = rect[1][0]
        ang = 90-rect[2]
    show_contour(black_img,rect,contours)

if __name__=="__main__":
    img_msg=cv2.imread("scripts/temporary/depth_2698.exr",cv2.IMREAD_UNCHANGED)#depth_458.exr
    # print(img_msg[200,270])

    # img_msg = img_msg[:,:,:3]
    # low=np.asarray([255,0,0])
    # up=np.asarray([255,0,0])
    # mask=cv2.inRange(img_msg,low,up)

    # ret,threshold_output=cv2.threshold( img_msg, 250, 255, cv2.THRESH_BINARY)
    # contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    # img=cv2.drawContours(img_msg, contours, -1, (0,0,255), 3)
    # # print(threshold_output[1][200,270])
    # cv2.imshow("1",img_msg)
    # cv2.waitKey()
    show_eye_img(img_msg)
    # get_obb(img_msg[:,:,:3])

    # cv2.imshow("1",img_msg)
    # cv2.waitKey()