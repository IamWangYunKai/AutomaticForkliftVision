#include "stdafx.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv){

	pcl::PCLPointCloud2::Ptr origin_cloud(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered_1(new pcl::PCLPointCloud2());
	//点云对象的读取
	pcl::PCDReader reader;
	reader.read("test.pcd", *origin_cloud);    //读取点云到cloud中
	std::cerr << "PointCloud before filtering: " << origin_cloud->width * origin_cloud->height
		<< " data points (" << pcl::getFieldsList(*origin_cloud) << ").";

	/*********************************************************************************
	创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
	**********************************************************************************/
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor_1;  //创建滤波对象
	sor_1.setInputCloud(origin_cloud);            //设置需要过滤的点云给滤波对象
	sor_1.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	sor_1.filter(*cloud_filtered_1);           //执行滤波处理，存储输出
	std::cerr << "PointCloud after filtering: " << cloud_filtered_1->width * cloud_filtered_1->height
		<< " data points (" << pcl::getFieldsList(*cloud_filtered_1) << ").";

	/*******************************************************************************
	statistical Outlier Removal
	*******************************************************************************/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud_filtered_1, *cloud_2);//把PCLPointCloud2转换为PointXYZ

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	//个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_2;   //创建滤波器对象
	sor_2.setInputCloud(cloud_2);                           //设置待滤波的点云
	sor_2.setMeanK(100);                               //设置在进行统计时考虑查询点临近点数
	sor_2.setStddevMulThresh(0.1);                      //设置判断是否为离群点的阀值
	sor_2.filter(*cloud_filtered_2);                    //存储

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered_2 << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("inliers.pcd", *cloud_filtered_2, false);

	//sor_2.setNegative(true);
	//sor_2.filter(*cloud_filtered_2);
	//writer.write<pcl::PointXYZ>("outliers.pcd", *cloud_filtered_2, false);



	/******************************************************************************
	写滤波后的数据
	*******************************************************************************/
	/*
	pcl::PCDWriter final_writer;
	final_writer.write("output.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
	*/
	return (0);
}