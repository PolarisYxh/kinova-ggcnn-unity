/**\
save
topic2_name = "/ggcnn/img/grasp_plain"; //topic 名称
topic1_name = "/ggcnn/img/width";
topic3_name = "/ggcnn/img/ang"; //topic 名称
 */

#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>//将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include <sensor_msgs/image_encodings.h>//头文件sensor_msgs/Image是ROS下的图像的类型，这个头文件中包含对图像进行编码的函数

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

#include <X11/Xlib.h>
using namespace std;
using namespace cv;

Mat rgb, rgb1, depth,depth1;
char successed_flag1 =0,successed_flag2=0,successed_flag3=0;

string topic2_name = "/ggcnn/img/grasp_plain"; //topic 名称
string topic1_name = "/ggcnn/img/width";
string topic3_name = "/ggcnn/img/ang"; //topic 名称

string save_imagedata = "/home/yxh/image/";

bool display_IMU5211( unsigned char buf[21] ,timeval time_stamp,string &out_result);
void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);
void  callback_function_color(const sensor_msgs::Image::ConstPtr  image_data)
{
   cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例

   pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
   pCvImage->image.copyTo(rgb);
    successed_flag1 = 1;
}
void  callback_function_color1(const sensor_msgs::Image::ConstPtr  image_data)
{
   cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例

   pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
   pCvImage->image.copyTo(rgb1);
    successed_flag3 = 1;
}
void  callback_function_depth(const sensor_msgs::Image::ConstPtr  image_data)
{
   cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
   pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
   pCvImage->image.copyTo(depth);
   successed_flag2=1;
}
void save_depth(const string& rgb_str,Mat d)
{
    double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;
    minMaxIdx(d, minp, maxp);
    Mat minMat(d.size(), CV_32FC1, *minp);//2, 2, CV_8UC3, Scalar(0,255,0)
    Mat diffMat = d-minMat;
    diffMat.convertTo(diffMat, CV_8U, 255.0/(maxv-minv));
    flip(diffMat, diffMat, 0);
    imwrite(rgb_str,diffMat);
}
int main(int argc,char** argv)
{
    string out_result;
    namedWindow("image color",CV_WINDOW_AUTOSIZE);
    namedWindow("image depth",CV_WINDOW_AUTOSIZE);
    ros::init(argc,argv,"kinect2_listen");
    if(!ros::ok())
            return 0;
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe(topic1_name,50,callback_function_color);
    ros::Subscriber sub3 = n.subscribe(topic3_name,50,callback_function_color1);
    ros::Subscriber sub2 = n.subscribe(topic2_name,50,callback_function_depth);
    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();
    string rgb_str, rgb1_str, dep_str;

   struct timeval time_val;
   struct timezone tz;
   double time_stamp;


    int num1=0;
    while(ros::ok())
    {
            if(successed_flag1&&successed_flag2&&successed_flag3)
            {
                //gettimeofday(&time_val,&tz);//us
				//  time_stamp =time_val.tv_sec+ time_val.tv_usec/1000000.0;
                // ostringstream os_rgb;
                //os_rgb<<time_val.tv_sec<<"."<<time_val.tv_usec;
                double k=0.000001*num1;
                 //rgb_str = save_imagedata+"rgb/"+os_rgb.str()+".png";
                rgb_str = save_imagedata+"rgb/"+std::to_string(k)+".png";
                rgb1_str = save_imagedata+"rgb1/"+std::to_string(k)+".png";
                dep_str =save_imagedata+"depth/"+std::to_string(k)+".png";// 输出图像目录
                // cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
                flip(rgb, rgb, 0);
                imwrite(rgb_str,rgb);

                save_depth(rgb1_str, rgb1);
                // cv::cvtColor(rgb1, rgb1, cv::COLOR_RGB2BGR);
                flip(rgb1, rgb1, 0);
                // imwrite(rgb1_str,rgb1);

                flip(depth, depth, 0);
                imwrite(dep_str,depth);
                // cv::cvtColor(depth, depth, cv::COLOR_RGB2BGR);
                // flip(depth, depth, 0);
                // imwrite(rgb1_str,depth);
                // save_depth(rgb_str, rgb);


                successed_flag1=0;
                successed_flag2=0;
                successed_flag3=0;

                cout<<"rgb -- time:  " <<  time_val.tv_sec<<"."<<time_val.tv_usec<<endl;
				cout<<"depth -- time:" <<  time_val.tv_sec<<"."<<time_val.tv_usec<<endl;
                num1++;
            }
    }

    ros::waitForShutdown();
    ros::shutdown();

    return 0;
}
