#include "ros/ros.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
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
#define fx 382.05
#define fy 382.05
#define x0 318.26
#define y0 238.32


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
    void time_Median(const Mat &src, Mat &mask2, Mat &mask3,Mat &out);
    Mat cameraFeed;
    Mat MedianFliter;
    Mat HSV;
    Mat threshold_ori;
    Mat mask1;
    Mat mask2;
    Mat mask3;
    ros::Subscriber rgb_sub; 
};
void talker_listener::modifyMat(Mat& image)  
{
    uint16_t array[5];
    // printf("image.rows:%d image.cols:%d/n",image.rows,image.cols);
    int w=0,s=0;
    for(int i=0;i<image.rows;i++) 
    {  
        for(int j=0;j<image.cols;j++)  
        {
            if(image.at<uint16_t>(i,j)==0){
                image.at<uint16_t>(i,j)=UINT16_MAX;
            }
            else{
                // image.at<uint16_t>(i,j)=image.at<uint16_t>(i,j)*P_y(i);
            }
            // image.at<uint16_t>(i,j)=100;

        }  
    }  
}
uint16_t talker_listener::Median1(const Mat &src,int i,int j,int w) {
	uint16_t arr[w];
    int w_radius=w/2;
    int w_start=i-w_radius;
    int w_end=i+w_radius;
    int w_prt=w_start;
    for(w_prt;w_prt<w_end;++w_prt){
        if(w_prt<0||w_prt>src.rows){
           int arr_prt=w_prt-w_start;
        //    cout<<"arr_prt  "<<arr_prt<<endl;
           arr[arr_prt]= UINT16_MAX;
        //    cout<<"arr[arr_prt]  "<<arr[arr_prt]<<endl;
        }
        else{
            arr[w_prt-w_start]=src.at<uint16_t>(w_prt, j);} 
    }
    std::sort(arr,arr+w);
	return arr[w/2+1];//返回中值
}
void talker_listener::MedianFlitering(const Mat &src, Mat &dst, int w, int s){
	if (!src.data)return;
	Mat _dst(src.rows/s+1,src.cols, src.type());
    int n=0;
    for (int j=0; j < src.cols; j=j+1)
	{
        // std::cout<<"第"<<j<<"列"<<std::endl;
		for (int i=0; i < src.rows; i=i+s) {
            _dst.at<uint16_t>(n, j)=Median1(src,i,j,w);
            // std::cout<<"第"<<n<<"行"<<std::endl;
            n++;
		}
        n=0;
    }
    // cv::imshow("Result", _dst);
	_dst.copyTo(dst);//拷贝
}
double talker_listener::P_y(double v){
  double Tan_angle_y,proportion_y;
  Tan_angle_y=(v-y0)/fx;//计算tan y
  proportion_y=cos(Pitch_angle-atan(Tan_angle_y))/cos(atan(Tan_angle_y));
  return proportion_y;
}
void talker_listener::time_Median(const Mat &src, Mat &mask2, Mat &mask3, Mat &out){
    int i,j;
    int dx1,dx2;
    src.copyTo(out);//拷贝
    for(i;i<mask2.rows;i++){
            for(j;j<mask2.rows;j++){
            dx1=abs(mask2.at<u_int16_t>(i,j)-mask2.at<u_int16_t>(i,j));
            dx2=abs(mask2.at<u_int16_t>(i,j)-mask3.at<u_int16_t>(i,j));
            if(dx1>300 && dx2>300){
                out.at<u_int16_t>(i,j)=UINT16_MAX;
            }
    }
    }
}
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
    // modifyMat(cameraFeed);
    MedianFlitering(cameraFeed,MedianFliter,100,10);
    Mat out;
    if(mask2.data){
        mask1=mask2;
    }
    if(mask3.data){
        mask2=mask3;
    }
    mask3=MedianFliter;
    if(mask1.data&&mask2.data&&mask3.data){
        std::cout<<"time_Median 图片处理中"<<std::endl;
        time_Median(mask2,mask1,mask3,out);
    }
    if(out.data){
        //show frames
        cv_bridge::CvImage cvi;
        int length;
        length=sizeof(msg.data)/msg.step;  //数组占内存总空间，除以单个元素占内存空间大小
        // printf("length of data=%d\n",msg.data);
        cvi.image = out;
        cv::imshow("WINDOW",out);
        cv::waitKey(3);
        sensor_msgs::Image im;
        sensor_msgs::CameraInfo info;
        cvi.toImageMsg(im);
        im.encoding=msg.encoding;
        im.header=msg.header;
        im.height=out.rows;
        im.is_bigendian=msg.is_bigendian;
        im.step=msg.step;
        im.width=msg.width;
        //相机信息
        pub.publish(im);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    talker_listener tl;
    ros::spin();
    return 0;
}