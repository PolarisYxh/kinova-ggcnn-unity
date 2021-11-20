import cv2
import numpy as np
import glob
import os
def show_contour(black_img,rect,contours):
    points=cv2.boxPoints(rect)

    for j in range(0,4):
        x=(int(points[j][0]),int(points[j][1]))
        y=(int(points[(j+1)%4][0]),int(points[(j+1)%4][1]))
        cv2.line( black_img, x, y, (0,0,255), 1, 8 )
    # cv2.drawContours(black_img, contours, -1, (0,0,255), 3)
    cv2.imshow("1",black_img)
    cv2.waitKey()
def change_01(points,change):
    if change:
        points[[1,3],:]=points[[3,1],:]
        # t=np.copy(points[3])
        # points[3]=points[1]
        # points[1]=t
    return points
def get_obb(rgb):
    '''get_obb from segmentation
    movableMask green
        handle blue

        "x": 268.0,
        "y": 197.0,
        "width": 3.0,
        "height": 6.0
    '''
    rgb = rgb[:,:,:3]
    low=np.asarray([255,0,0])
    up=np.asarray([255,0,0])
    mask=cv2.inRange(rgb,low,up)
    black_img = np.zeros(rgb.shape,np.uint8)
    # ret,threshold_output=cv2.threshold( rgb, 250, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours)==0:
        return None
    ans = []
    rect = cv2.minAreaRect(contours[0])
    points = cv2.boxPoints(rect)
    center = [rect[0][0],rect[0][1]]
    bili = np.linalg.norm(points[0]-points[1])/np.linalg.norm(points[0]-points[3])
    has_tochange=False
    if bili>1:
        y=points[3]
        has_tochange=True
    else:
        y=points[1]
        bili = 1/bili
    delta = y-points[0]
    delta = [delta[1],-delta[0]]
    m=min(rect[1][0],rect[1][1])
    for i in range(1, int(int(bili+1)/2)):
        c1 = [center[0]+i*delta[0],center[1]+i*delta[1]]
        rec=(c1,(m,m),rect[2])
        ans.append(change_01(cv2.boxPoints(rec),has_tochange))
        c2 = [center[0]-i*delta[0],center[1]-i*delta[1]]
        rec=(c2,(m,m),rect[2])
        ans.append(change_01(cv2.boxPoints(rec),has_tochange))
    if int(int(bili+1)/2)>1:
        rec = (center,(m,m),rect[2])
        ans.append(change_01(cv2.boxPoints(rec),has_tochange))
    else:
        ans.append(change_01(points,has_tochange))
    # show_contour(black_img,rect,contours)
    return ans

if __name__=="__main__":
    file_path = "/media/yxh/My Passport/graduateDesign/handleMovableDataSet/handledataset/"
    graspf = glob.glob(os.path.join(file_path, '*/*/*', 'segmentation_*.png'))
    for x in graspf:
        img=cv2.imread(x,cv2.IMREAD_UNCHANGED)
        try:
            points = get_obb(img)
        except:
            continue
        if points is None:
            continue
        y=os.path.dirname(x)
        basename = os.path.basename(x)[13:-4]
        with open(y+'/obb_%s.txt'%basename,'w') as file:
            for r in points:
                x=str(r[0].tolist())
                x=x[1:-1]
                file.write(x+'\n')

                x=str(r[1].tolist())
                x=x[1:-1]
                file.write(x+'\n')

                x=str(r[2].tolist())
                x=x[1:-1]
                file.write(x+'\n')

                x=str(r[3].tolist())
                x=x[1:-1]
                file.write(x+'\n')
