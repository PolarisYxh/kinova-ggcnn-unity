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
char successed_flag1 =0;

string topic1_name = "/camera/depth/image_meters"; //topic 名称
string save_imagedata = "/home/yxh/image/";
bool app_stopped = false;

void sigint_handler(int sig){
	if(sig == SIGINT){
		// ctrl+c退出时执行的代码
		std::cout << "ctrl+c pressed!" << std::endl;
		app_stopped = true;
	}
}
void  callback_function_color( const sensor_msgs::Image::ConstPtr  image_data);
int main(int argc,char** argv)
{
    //namedWindow("image color",CV_WINDOW_AUTOSIZE);
    std::cout<<"init start"<<std::endl;
    signal(SIGINT, sigint_handler);
    std::cout<<"init start"<<std::endl;
    ros::init(argc,argv,"rgb_listen");
    std::cout<<"init ok"<<std::endl;
    if(!ros::ok())
        return 0;
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe(topic1_name,50,callback_function_color);
    ros::AsyncSpinner spinner(2); // Use 3 threads
    spinner.start();
    string rgb_str, dep_str;

    struct timeval time_val;
    struct timezone tz;
    double time_stamp;


    int num1=0;
    while(ros::ok())
    {
        if (app_stopped){
			break;
		}
        if(successed_flag1)
        {
            double k=0.000001*num1;
                //rgb_str = save_imagedata+"rgb/"+os_rgb.str()+".png";
            rgb_str = save_imagedata+"rgb/"+std::to_string(k)+".png";
            //cv::Mat rgbmat;
            //cv::cvtColor(rgb, rgbmat, cv::COLOR_RGB2BGR);
            double minv = 0.0, maxv = 0.0;
            double* minp = &minv;
            double* maxp = &maxv;
            minMaxIdx(rgb, minp, maxp);
            rgb.convertTo(rgb, CV_8U, 255.0/(maxv-minv));
            flip(rgb, rgb, 0);
            imwrite(rgb_str,rgb);

            successed_flag1 = 0;
            cout<<"rgb -- time:  " <<  time_val.tv_sec<<"."<<time_val.tv_usec<<endl;
            num1++;
        }
    }

    ros::waitForShutdown();
    ros::shutdown();

    return 0;
}
void  callback_function_color(const sensor_msgs::Image::ConstPtr  image_data)
{
   cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例

   pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
   pCvImage->image.copyTo(rgb);
   successed_flag1 = 1;
   //cv::waitKey(0);
}