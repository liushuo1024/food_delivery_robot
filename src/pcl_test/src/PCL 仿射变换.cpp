#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>  //  pcl::transformPointCloud 用到这个头文件
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int
main(int argc, char** argv) {

	// 加载点云数据文件
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile("飞机.pcd", *cloud);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();//初始化变换矩阵为单位矩阵
	// 在 X 轴上定义一个 1米的平移.
	transform.translation() << 1.0, 0.0, 0.0;

	// 旋转; X轴旋转45°, Y轴上旋转0°,Z轴上旋转0°
	float angle_x = 45;
	float angle_y = 0;
	float angle_z = 0;

	// 国际单位制中，弧度是角的度量单位，Eigen中也是以弧度作为角度量单位，
	// 因此需要将角度值转换为弧度制
	transform.rotate(Eigen::AngleAxisf(pcl::deg2rad(angle_x), Eigen::Vector3f::UnitX()));
	transform.rotate(Eigen::AngleAxisf(pcl::deg2rad(angle_y), Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(pcl::deg2rad(angle_z), Eigen::Vector3f::UnitZ()));
	// 打印变换矩阵
	cout << "变换矩阵为：\n" << transform.matrix() << endl;

	// 执行变换，并将结果保存在新创建的 transformed_cloud 中
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
	//pcl::io::savePCDFileASCII("tg.pcd", *transformed_cloud);

	cout << "原始点云显示为白色，变换后的点云为红色" << endl;
	pcl::visualization::PCLVisualizer viewer("Affine3f transformation example");

	// 为点云定义 R,G,B 颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud, 255, 255, 255);
	// 输出点云到查看器，使用颜色管理
	viewer.addPointCloud(cloud, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 255, 0, 0); // 红
	viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // 设置背景为深灰
	viewer.addText("The original point cloud is white,the transformed point cloud is red", 20, 20, "text");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud");
	//viewer.setPosition(800, 400); // 设置窗口位置
	// 启动可视化
    viewer.addCoordinateSystem(0.2);  //显示XYZ指示轴
    //viewer.initCameraParameters();   //初始化摄像头参数


	while (!viewer.wasStopped()) { // 在按下 "q" 键之前一直会显示窗口
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return 0;
}
