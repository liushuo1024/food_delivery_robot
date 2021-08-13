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

#define PI 3.14159265
#define Pitch_angle PI/6
#define fx 421.84
#define fy 421.84
#define x0 422.08
#define y0 238.14


using namespace cv;
class talker_listener
{
private:
    int num;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
public:
    talker_listener()
    {
        num = 100;
        sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &talker_listener::Callback, this);
        pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);
        cv::namedWindow("WINDOW");
    }
    double P_y(double v);
    void modifyMat(Mat& image);
    void Callback(const sensor_msgs::Image& msg);
    uint16_t Median(uint16_t n1, uint16_t n2, uint16_t n3, uint16_t n4, uint16_t n5,
	                uint16_t n6, uint16_t n7, uint16_t n8, uint16_t n9);
    uint16_t Median1(const Mat &src,int i,int j,int w); 
    void MedianFlitering(const Mat &src, Mat &dst,int w=9,int s=1);
    Mat cameraFeed;
    Mat MedianFliter;
    Mat HSV;
    Mat threshold_ori;
    ros::Subscriber rgb_sub; 
};