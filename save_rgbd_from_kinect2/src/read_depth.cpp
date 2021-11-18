/**
 *
 * 函数功能：采集iaikinect2输出的彩色图和深度图数据，并以文件的形式进行存储
 *
 *
 * 分隔符为　逗号'，'　　
 * 时间戳单位为秒(s)　精确到小数点后６位(us)
 *
 * maker:crp
 * 2017-5-13
 */

#include <iostream>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>//将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include <sensor_msgs/image_encodings.h>//头文件sensor_msgs/Image是ROS下的图像的类型，这个头文件中包含对图像进行编码的函数

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <fstream>

#include <signal.h>

using namespace std;
using namespace cv;

Mat rgb;

string save_imagedata = "/home/yxh/image/";
bool app_stopped = false;

void sigint_handler(int sig){
	if(sig == SIGINT){
		// ctrl+c退出时执行的代码
		std::cout << "ctrl+c pressed!" << std::endl;
		app_stopped = true;
	}
}
int main(int argc,char** argv)
{
    //namedWindow("image color",CV_WINDOW_AUTOSIZE);
    std::cout<<"init start"<<std::endl;
    signal(SIGINT, sigint_handler);
    string rgb_str, dep_str;

    cv::Mat rgb=cv::imread("/home/yxh/Documents/mars_unity/Unity-Robotics-Hub-main/tutorials/pick_and_place/ROS/src/save_rgbd_from_kinect2/depth_22.png");
    //cv::cvtColor(rgb, rgbmat, cv::COLOR_RGB2BGR);
    double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;
    minMaxIdx(rgb, minp, maxp);
    cout<<*minp<<" "<<*maxp;
    // Mat minMat(rgb.size(), CV_32FC1, *minp);//2, 2, CV_8UC3, Scalar(0,255,0)
    // Mat diffMat = rgb-minMat;
    // diffMat.convertTo(diffMat, CV_8U, 255.0/(maxv-minv));
    // flip(diffMat, diffMat, 0);
    // imwrite(rgb_str,diffMat);

    return 0;
}