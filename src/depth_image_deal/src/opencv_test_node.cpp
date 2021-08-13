#include "ros/ros.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>      /* printf */ 
#include <iostream>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>       /* cos */
#include <algorithm>
#include <pthread.h>
using namespace cv;

//template <typename TYPE, void (TYPE::*_RunThread)() >  
// void* _thread_t(void* param)//线程启动函数，声明为模板函数  
// {     
//  TYPE* This = (TYPE*)param;     
//  This->image_show();     
//  return NULL;  
//  }

class talker_listener
{
private:
    int num;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

public:
    void Callback(const sensor_msgs::Image& msg);
    Mat cameraFeed;
    // Mat g_dstImgMedian((int)480, (int)640, CV_8UC1);
    Mat threshold_ori;
    ros::Subscriber rgb_sub;
    static  void * insert_pth(void*);
    int g_MedianBlurVal = 12; 
    talker_listener()
    {
        num = 100;
        sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &talker_listener::Callback, this);
        pub = nh.advertise<sensor_msgs::Image>("/camera/depth/my_image_raw", 1); 
        cv::namedWindow("WINDOW");
        // pthread_t tid;
        // pthread_create(&tid,NULL,insert_pth,(void*)this);
    }
    // static void on_MedianBlur(int, void *)
    // {
    //    medianBlur(g_srcImage, g_dstImgMedian, g_MedianBlurVal * 2 + 1);
    //    imshow("【中值滤波】", g_dstImgMedian);
    // }
};
void talker_listener::Callback(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); // Caution the type here.
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("cv_bridge exception in rgbcallback: %s", ex.what());
        exit(-1);
    }
    cameraFeed = cv_ptr->image.clone();
    Mat g_dstImgMedian = Mat(cameraFeed.rows,cameraFeed.cols, CV_16UC1, Scalar(255));
    cameraFeed.copyTo(g_dstImgMedian);
    // medianBlur(g_dstImgMedian, cameraFeed, 9);
    //show frames
    cv_bridge::CvImage cvi;
    int length;
    length=sizeof(msg.data)/msg.step;  //数组占内存总空间，除以单个元素占内存空间大小
    // printf("length of data=%d\n",msg.data);
    cvi.image = g_dstImgMedian;
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    im.encoding=msg.encoding;
    im.header=msg.header;
    im.height=msg.height;
    im.is_bigendian=msg.is_bigendian;
    im.step=msg.step;
    im.width=msg.width;
    cv::imshow("WINDOW",g_dstImgMedian);
    cv::imwrite("/home/liushuo/opencv_ws/my_image.png",g_dstImgMedian);
    cv::waitKey(3);
    pub.publish(im);
    // std::thread thread(&test, this);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    talker_listener tl;
    ros::spin();
    return 0;
}
