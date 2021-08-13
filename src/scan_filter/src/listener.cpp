#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <fstream>
#include "ros/ros.h"
#include <math.h>
#include <numeric>
#include <iostream>
#include <iterator>
class laser_listener
{
private:
    int num;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    sensor_msgs::LaserScan scan_buff_new;
    sensor_msgs::LaserScan scan_buff_old;
    sensor_msgs::LaserScan scan_buff_real;
    bool flag=false;
public:
    laser_listener()
    {
        num = 100;
        sub = nh.subscribe("/camera_scan", 1, &laser_listener::Callback, this);
        pub = nh.advertise<sensor_msgs::LaserScan>("/filter_scan", 1);
    }
    void scan_space_filter(std::vector<float> &ranges,float dx);
    void scan_time_filter(float my_dx);
    void Callback(const sensor_msgs::LaserScan &msg);
};

void laser_listener::Callback(const sensor_msgs::LaserScan &msg)
{
    int j;
    if(flag==false){
    for(j = 0; j<800; j++)
    {
        // std::cout<<"开始"<< j << "数据"<< msg.ranges[j]<<std::endl;
        // scan_buff_new.ranges[j] = msg.ranges[j];
        scan_buff_new.ranges.insert(scan_buff_new.ranges.begin()+j,msg.ranges[j]);
        scan_buff_old.ranges.insert(scan_buff_old.ranges.begin()+j,msg.ranges[j]);
    }
    }
    if(flag==true){
        // std::cout<<"开始"<<msg.ranges[1]<<std::endl;
        int i;
        for(i = 0; i<800; i++)
        {
          // std::cout<<"开始"<< i <<std::endl;
            // scan_buff_old.ranges.erase(scan_buff_old.ranges.begin()+i);
            // scan_buff_old.ranges.insert(scan_buff_old.ranges.begin()+i,scan_buff_new.ranges[i]);
            scan_buff_new.ranges.erase(scan_buff_new.ranges.begin()+i);
            scan_buff_new.ranges.insert(scan_buff_new.ranges.begin()+i,msg.ranges[i]);
        }
        scan_time_filter(0.5);
        // std::cout<<"数组"<<msg.ranges[1]<<std::endl;
        // std::cout<<"数组大小"<<msg.ranges.size()<<std::endl;
        float dx;
        scan_buff_real.header=msg.header;
        scan_buff_real.angle_increment=msg.angle_increment;
        scan_buff_real.angle_max=msg.angle_max;
        scan_buff_real.angle_min=msg.angle_min;
        scan_buff_real.intensities=msg.intensities;
        scan_buff_real.range_max=msg.range_max;
        scan_buff_real.range_min=msg.range_min;
        scan_buff_real.scan_time=msg.scan_time;
        scan_buff_real.time_increment=msg.time_increment;
        pub.publish(scan_buff_real);
        scan_buff_real.ranges.erase(scan_buff_real.ranges.begin(),scan_buff_real.ranges.end());
        int w;
        for(w = 0; w<800; w++)
        {
            // std::cout<<"开始"<< w <<std::endl;
            scan_buff_old.ranges.erase(scan_buff_old.ranges.begin()+w);
            scan_buff_old.ranges.insert(scan_buff_old.ranges.begin()+w,scan_buff_new.ranges[w]);
        }
        }
    flag=true;
}

void laser_listener::scan_space_filter(std::vector<float> &ranges,float dx)
{
    std::vector<float>::iterator it;
    it = ranges.begin();
    it=it+4;
    for(it; it!=(ranges.end()-4); it++)
    {
      float d_left,d_right,average_left,average_right;
      average_left=*(it-1)+*(it-2)+*(it-3)+*(it-4);
      average_right=*(it+1)+*(it+2)+*(it+3)+*(it+4);
      d_left=fabs(*it-average_left);
      d_right=fabs(*it-average_right);
      if(d_left>dx && d_right>dx){
      }

    }
}
void laser_listener::scan_time_filter(float my_dx)
{
    int i,j;
    float dx;
    for(i = 0; i<800; i++)
    {
        dx=fabs(scan_buff_new.ranges[i]-scan_buff_old.ranges[i]);
        std::cout<<"dx"<< dx <<std::endl;
        if(dx>my_dx){
            // scan_buff_new.ranges.erase(scan_buff_new.ranges.begin()+i);
            // scan_buff_new.ranges.insert(scan_buff_new.ranges.begin()+i,5);
            scan_buff_real.ranges.insert(scan_buff_real.ranges.begin()+i,NULL);
        }
        else{
            std::cout<<"123456"<<std::endl;
            scan_buff_real.ranges.insert(scan_buff_real.ranges.begin()+i,scan_buff_new.ranges[i]);
        }
        }
    }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_listener");
    laser_listener tl;
    ros::spin();
    return 0;
}


