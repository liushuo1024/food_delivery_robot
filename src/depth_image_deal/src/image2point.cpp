#include "space_filter.h"
//(u-cx)point[2]/fx
talker_listener::talker_listener(){
        num = 3;
        sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &talker_listener::Callback, this);
        pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 100);
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
//深度图按列进行中值滤波
void talker_listener::MedianFlitering(const Mat &src, Mat &dst, int w, int s){
	if (!src.data)return;
	Mat _dst(src.size(), src.type());
    for (int j=0; j < src.cols; j=j+1)
	{
        // cout<<"第"<<j<<"列"<<endl;
		for (int i=0; i < src.rows; i=i+s) {
            int n=i+1;
            for (n; n < i+s; n=n+1) {
               _dst.at<uint16_t>(n, j)=UINT16_MAX;
            }
            _dst.at<uint16_t>(i, j)=Median1(src,i,j,w);
		}
    }
	_dst.copyTo(dst);//拷贝
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
    Mat MedianFlitering_image;
    MedianFlitering(rectify_image,MedianFlitering_image,10,1);
    //show frames
    cv_bridge::CvImage cvi;
    int length;
    length=sizeof(msg.data)/msg.step;  //数组占内存总空间，除以单个元素占内存空间大小
    cvi.image = space_filter_image;
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
    talker_listener tl;
    ros::spin();
    return 0;
}