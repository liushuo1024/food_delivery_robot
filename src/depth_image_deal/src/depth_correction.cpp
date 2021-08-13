#include "depth_correction.h"
depth_correction::depth_correction(){
    num = 100;
    sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &depth_correction::Callback, this);
    pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);
}
void depth_correction::modifyMat(Mat& image)  
{
    for(int i=0;i<image.rows;i++) 
    {  
        for(int j=0;j<image.cols;j++)  
        {
            if(image.at<uint16_t>(i,j)==0){
                image.at<uint16_t>(i,j)=UINT16_MAX;
            }
            else{
                image.at<uint16_t>(i,j)=image.at<uint16_t>(i,j)*P_y(i);
            }
        }  
    }  
}
double depth_correction::P_y(double v){
  double Tan_angle_y,proportion_y;
  Tan_angle_y=(v-y0)/fx;//计算tan y
  proportion_y=cos(Pitch_angle-atan(Tan_angle_y))/cos(atan(Tan_angle_y));
  return proportion_y;
}
void depth_correction::Callback(const sensor_msgs::Image& msg)
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
    modifyMat(cameraFeed);
    //show frames
    cv_bridge::CvImage cvi;
    cvi.image = cameraFeed;
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    im.encoding=msg.encoding;
    im.header=msg.header;
    im.height=msg.height;
    im.is_bigendian=msg.is_bigendian;
    im.step=msg.step;
    im.width=msg.width;
    pub.publish(im);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    depth_correction tl;
    ros::spin();
    return 0;
}
