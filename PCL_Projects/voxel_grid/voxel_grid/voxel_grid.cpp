#include "stdafx.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main(int argc, char** argv)
{

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	//点云对象的读取
	pcl::PCDReader reader;

	reader.read("test.pcd", *cloud);    //读取点云到cloud中

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ").";

	/******************************************************************************
	创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
	**********************************************************************************/
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //创建滤波对象
	sor.setInputCloud(cloud);            //设置需要过滤的点云给滤波对象
	sor.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	sor.filter(*cloud_filtered);           //执行滤波处理，存储输出

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ").";

	pcl::PCDWriter writer;
	writer.write("output.pcd", *cloud_filtered,
		Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

	return (0);
}