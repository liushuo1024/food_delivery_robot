#include "rectify_depth_image.h"
#include <sensor_msgs/CameraInfo.h>
//(u-cx)point[2]/fx
talker_listener::talker_listener(){
        num = 3;
        sub = nh.subscribe("/camera/depth/image", 1, &talker_listener::Callback, this);
        camera_info = nh.subscribe("/camera/depth/camera_info", 1, &talker_listener::camera_info_CB, this);
        pub = nh.advertise<sensor_msgs::Image>("/camera/depth/my_image_raw", 100);
    }
//去除定高，单位毫米
void talker_listener::space_filter(const Mat &src, Mat &out, float height_limit){
    src.copyTo(out);
    for(int i=0;i<out.rows;i++) 
    {  
        for(int j=0;j<out.cols;j++)  
        {
            double height;
            height=out.at<uint16_t>(i,j)*P_y(i);
            if(height>height_limit){
                out.at<uint16_t>(i,j)=NULL;
            }
        }  
    }
}

double talker_listener::P_y(double v){
  double Tan_angle_y,proportion_y;
  Tan_angle_y=(v-y0)/fx;//计算tan y
  proportion_y=sin(Pitch_angle-atan(Tan_angle_y))/cos(atan(Tan_angle_y));
  return proportion_y;
}
//深度矫正
void talker_listener::modifyMat(const Mat &src, Mat &out){
    src.copyTo(out);
    for(int i=0;i<out.rows;i++) 
    {  
        for(int j=0;j<out.cols;j++)  
        {
            if(src.at<uint16_t>(i,j)==0){
                out.at<uint16_t>(i,j)=NULL;
            }
            else{
                out.at<uint16_t>(i,j)=src.at<uint16_t>(i,j)*P_z(i);
            }
        }  
    }
}

double talker_listener::P_z(double v){
  double Tan_angle_y,proportion_z;
  Tan_angle_y=(v-y0)/fx;//计算tan y
  proportion_z=cos(Pitch_angle-atan(Tan_angle_y))/cos(atan(Tan_angle_y));
  return proportion_z;
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
void talker_listener::camera_info_CB(const sensor_msgs::CameraInfo& msg)
{
    // [474.16741943359375, 0.0,                317.7526550292969,
    //  0.0,                474.16741943359375, 196.19100952148438,
    //  0.0,                0.0,                1.0]
    fx = msg.K[0];
    fy = msg.K[4];
    x0 = msg.K[2];
    y0 = msg.K[5];
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
    //去除定高，单位毫米
    Mat space_filter_image;
    space_filter(cameraFeed,space_filter_image,300);
    //深度矫正
    Mat rectify_image;
    modifyMat(space_filter_image,rectify_image);
    //show frames
    cv_bridge::CvImage cvi;
    cvi.image = rectify_image;
    cvi.encoding = msg.encoding;
    cvi.header = msg.header;
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    pub.publish(im);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    talker_listener tl;
    ros::spin();
    return 0;
}