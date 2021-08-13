#include "image_transform.h"
image_transform::image_transform()
{
    num = 100;
    sub = nh.subscribe("/camera/depth/image_rect_raw", 1, &image_transform::Callback, this);
    pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);
}
void image_transform::Callback(const sensor_msgs::Image& msg)
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
#include "MedianFlitering.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    image_transform tl;
    ros::spin();
    return 0;
}