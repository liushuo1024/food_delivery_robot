#include "pcl_test_core.h"
// #include <pcl/filters/extract_indices.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>
#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
// #include <sensor_msgs/PointCloud2.h>
void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZ> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
// #pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].y > clip_height)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}
void OutlierRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    //读入点云数据
    // std::cout << "Cloud before filtering:\n " << *cloud << std::endl;
    // -----------------统计滤波-------------------
    // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
    // 个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_in);   //设置待滤波的点云
    sor.setMeanK (50);           //设置在进行统计时考虑查询点邻近点数
    sor.setStddevMulThresh (1);  //设置判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
    //即：当判断点的k近邻平均距离(mean distance)大于全局的1倍标准差+平均距离(global distances mean and standard)，则为离群点。
    sor.filter (*cloud_out); //存储内点
    // std::cout << "Cloud after filtering: \n" <<  *cloud_filtered << std::endl;
}

void low_(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out){
	pcl::PCLPointCloud2::Ptr cloud_in2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
	toPCLPointCloud2(*cloud_in,*cloud_in2);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_in2);
	sor.setLeafSize(0.05f,0.05f,0.05f);
	sor.filter (*cloud_filtered);
	fromPCLPointCloud2(*cloud_filtered,*cloud_out);
}
PclTestCore::PclTestCore(ros::NodeHandle &nh){
    sub_point_cloud_ = nh.subscribe("/kinect2/hd/points",10, &PclTestCore::point_cb, this);

    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

    ros::spin();
}

PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){
    
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr low_point(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr real_point(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

	// Eigen::Affine3f transform = Eigen::Affine3f::Identity();//初始化变换矩阵为单位矩阵

	// // 旋转; X轴旋转45°, Y轴上旋转0°,Z轴上旋转0°
	// float angle_x = 0;
	// float angle_y = 0;
	// float angle_z = 0;

	// // 国际单位制中，弧度是角的度量单位，Eigen中也是以弧度作为角度量单位，
	// // 因此需要将角度值转换为弧度制
	// transform.rotate(Eigen::AngleAxisf(pcl::deg2rad(angle_x), Eigen::Vector3f::UnitX()));
	// transform.rotate(Eigen::AngleAxisf(pcl::deg2rad(angle_y), Eigen::Vector3f::UnitY()));
	// transform.rotate(Eigen::AngleAxisf(pcl::deg2rad(angle_z), Eigen::Vector3f::UnitZ()));

	// // 执行变换，并将结果保存在新创建的 transformed_cloud 中
	// pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// pcl::transformPointCloud(*current_pc_ptr, *transformed_cloud, transform);
	// //pcl::io::savePCDFileASCII("tg.pcd", *transformed_cloud);
    low_(current_pc_ptr,low_point);//点云降采样
    OutlierRemove(low_point,real_point);//移除离群点
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (real_point);            //设置输入点云
    pass.setFilterFieldName ("y");         //设置过滤时所需要点云类型的Z字段
    float limit_min,limit_max;
    limit_min=0.1;
    limit_max=10;
    pass.setFilterLimits (limit_min, limit_max);        //设置在过滤字段的范围
    pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
    pass.filter (*filtered_pc_ptr);            //执行滤波，保存过滤结果在cloud_filtered
    // clip_above(0.1,current_pc_ptr,filtered_pc_ptr);
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);

    pub_pc.header = in_cloud_ptr->header;
    // pub_pc.header.frame_id="camera_link";
    pub_filtered_points_.publish(pub_pc);
}
