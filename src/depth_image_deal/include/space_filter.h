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
#define fx 426.22
#define fy 426.22
#define x0 422.41
#define y0 238.30
// 421.84, 0.0,   422.08,
// 0.0,   421.84, 238.14,
// 0.0,    0.0,    1.0
//2号相机
// 426.22, 0.0, 422.41,
// 0.0, 426.22, 238.30, 
//0.0, 0.0, 1.0
using namespace cv;
class talker_listener
{
private:
    int num;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
public:
    talker_listener();
    Mat cameraFeed;
    Mat MedianFliter;
    double P_z(double v);
    double P_y(double v);
    void space_filter(const Mat &src, Mat &out, float height_limit);
    void modifyMat(const Mat &src, Mat &out);
    void Callback(const sensor_msgs::Image& msg);
    uint16_t Median1(const Mat &src,int i,int j,int w); 
    void MedianFlitering(const Mat &src, Mat &dst,int w=9,int s=1);
};