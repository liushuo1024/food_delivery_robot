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
class image_transform
{
private:

protected:
    int num;
    Mat cameraFeed;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
public:
    image_transform();
    virtual void Callback(const sensor_msgs::Image& msg);
};