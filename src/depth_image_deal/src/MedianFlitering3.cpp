#include "MedianFlitering.h"
talker_listener::talker_listener(){
        num = 100;
        sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &talker_listener::Callback, this);
        pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 100);
        cv::namedWindow("WINDOW");
    }
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
double talker_listener::P_y(double v){
  double Tan_angle_y,proportion_y;
  Tan_angle_y=(v-y0)/fx;//计算tan y
  proportion_y=cos(Pitch_angle-atan(Tan_angle_y))/cos(atan(Tan_angle_y));
  return proportion_y;
}
void talker_listener::time_Median(const Mat &mask_now, Mat &mask_old, Mat &out){
    std::cout<<"time_Median start"<<std::endl;
    u_int16_t dx_old;
    mask_now.copyTo(out);//拷贝
    int k,x;
	for(k=0;k<480;k++)
	{
		for(x=0;x<848;x++)
		{
            dx_old=abs(mask_now.at<u_int16_t>(k,x)-mask_old.at<u_int16_t>(k,x));
            // std::cout<<"图像行列"<<mask_now.rows<<mask_now.cols<<std::endl;
            // std::cout<<"循环到"<<k<<"行"<<std::endl;
            // std::cout<<"mask_now "<<mask_now.at<u_int16_t>(k,x)<<"mask_old "<<mask_old.at<u_int16_t>(k,x)<<"dx_old： "<<dx_old<<std::endl;
            if(dx_old>150){
                // std::cout<<"离群点数值的"<<k<<"行"<<x<<"列。"<<"值"<<dx_old<<std::endl;
                out.at<u_int16_t>(k,x)=UINT16_MAX;}
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
    if(mask_now.data){
        mask_old=mask_now;
    }
    mask_now=cameraFeed;
    if(mask_old.data&&mask_now.data){
        std::cout<<"time_Median 图片处理中"<<std::endl;
        time_Median(mask_now,mask_old,out);
        mask_old=out;
    }
    if(out.data){
        //show frames
        cv_bridge::CvImage cvi;
        int length;
        length=sizeof(msg.data)/msg.step;  //数组占内存总空间，除以单个元素占内存空间大小
        // printf("length of data=%d\n",msg.data);
        cvi.image = out;
        cv::imshow("WINDOW",out);
        cv::waitKey(1);
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
    std::cout<<"回调处理完了"<<std::endl;
}

