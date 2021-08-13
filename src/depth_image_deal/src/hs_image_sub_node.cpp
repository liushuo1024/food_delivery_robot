#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
 
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>       /* cos */

#define PI 3.14159265
#define Pitch_angle PI/6
#define fx 382.05
#define fy 382.05
#define x0 318.26
#define y0 238.32

using namespace std;
using namespace cv;
 
const string Original_winName = "Original Image";
const string Thresh_winName = "Threshed Image";

class DepthImagemodify
{
  public:
    DepthImagemodify(ros::NodeHandle& n);
    ~DepthImagemodify();
    void connectCb(ros::NodeHandle& n);
    double getTan(double v);
    // void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    ros::Publisher pub_; 
    Mat cameraFeed;
    Mat HSV;
    Mat threshold_ori;
    ros::Subscriber rgb_sub; 
};
void DepthImagemodify::connectCb(ros::NodeHandle& n){
    ros::Subscriber rgb_sub = n.subscribe("/camera/depth/image_rect_raw", 1,depthCallback);
    // ros::Subscriber sub = n.subscribe("chatter", 1000, P_y);
    // n.subscribe("/camera/depth/image_rect_raw", 1,P_y);
    //  n.subscriber("/camera/depth/image_rect_raw", 1,DepthImagemodify::depthCallback);
}
DepthImagemodify::DepthImagemodify(ros::NodeHandle& n){
    pub_ = n.advertise<sensor_msgs::Image>("/my/image_raw", 10, boost::bind(&DepthImagemodify::connectCb,n));
}
double getTan(double v){
  double y;
  y=(v-y0)/fx;
  return y;
}

double P_y(double v){
  double Tan_angle_y,proportion_y;
  Tan_angle_y=getTan(v);
//   printf ("Tan_angle_y is %f.\n", atan(Tan_angle_y));
  proportion_y=cos(Pitch_angle-atan(Tan_angle_y))/cos(atan(Tan_angle_y));
  // proportion_y=cos(Pitch_angle-PI/12);
  return proportion_y;
}
void modifyMat(Mat& image)  
{  
    for(int i=0;i<image.rows;i++)  
    {  
        for(int j=0;j<image.cols;j++)  
        {  
            image.at<uchar>(i,j)=image.at<uchar>(i,j)*P_y(j);
        }  
    }  
}   
void depthCallback(const sensor_msgs::ImageConstPtr& msg)
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
    // cameraFeed = cv_ptr->image.clone();
    // // cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
    // // inRange(HSV,Scalar(0,133,0),Scalar(21,256,256),threshold_ori);
    // modifyMat(cameraFeed);
    // //show frames
    //   cv_bridge::CvImage cvi;
    // cvi.header.stamp = time;
    // cvi.header.frame_id = "image";
    // cvi.encoding = "bgr8";
    // cvi.image = cv_image;
    
    // sensor_msgs::Image im;
    // cvi.toImageMsg(im);
    // image_pub.publish(im);
    // imshow(Original_winName,cameraFeed);
    // // imshow(Thresh_winName,threshold_ori);
    // //delay 10ms so that screen can refresh.
    // //image will not appear without this waitKey() command
    // waitKey(10);
 
}
 
void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
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
    // cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
    // inRange(HSV,Scalar(0,133,0),Scalar(21,256,256),threshold_ori);
    modifyMat(cameraFeed);
    //show frames
      cv_bridge::CvImage cvi;
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = cv_image;
    
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    image_pub.publish(im);
    imshow(Original_winName,cameraFeed);
    // imshow(Thresh_winName,threshold_ori);
    //delay 10ms so that screen can refresh.
    //image will not appear without this waitKey() command
    waitKey(10);
 
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "HornedSungemGrabber");
 
    ros::NodeHandle n;
    ros::Publisher pub_; 
    pub_ = n.advertise<sensor_msgs::Image>("/my/image_raw", 10, boost::bind(&connectCb));
//	topic name of HornedSungem
    // ros::Subscriber rgb_sub = n.subscribe("/camera/depth/image_rect_raw", 1, rgbCallback);
    ROS_INFO("Subscribe to the HS color image topic.");
 
    ros::spin();
    return 0;
}