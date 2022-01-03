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
    rect = cv2.minAreaRect(contours[0])

    points = cv2.boxPoints(rect)
    # show_contour(black_img,rect,contours)
    return points

if __name__=="__main__":
    file_path = "/media/yxh/My Passport/graduateDesign/handleMovableDataSet/handledataset/"
    graspf = glob.glob(os.path.join(file_path, '*/*/*', 'segmentation_*.png'))
    for x in graspf:
        img=cv2.imread(x,cv2.IMREAD_UNCHANGED)
        r = get_obb(img)
        if r is None:
            continue
        y=os.path.dirname(x)
        basename = os.path.basename(x)[13:-4]
        with open(y+'/obb_%s.txt'%basename,'w') as file:

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
