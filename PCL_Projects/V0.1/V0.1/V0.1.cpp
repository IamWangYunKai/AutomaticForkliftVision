#include "stdafx.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/gp3.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <gl/gl.h>
#include <gl/glu.h>
#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

int main() {
	pcl::PCLPointCloud2::Ptr input_cloud(new pcl::PCLPointCloud2());
	//点云对象的读取
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	reader.read("4.pcd", *input_cloud);


	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

	/*********************************************************************************
	创建一个叶大小为1cm的pcl::VoxelGrid滤波器，
	**********************************************************************************/
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor_1;  //创建滤波对象
	sor_1.setInputCloud(input_cloud);            //设置需要过滤的点云给滤波对象
	sor_1.setLeafSize(0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
	sor_1.filter(*cloud_filtered);           //执行滤波处理，存储输出

	/*******************************************************************************
	statistical Outlier Removal
	*******************************************************************************/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_2);//把PCLPointCloud2转换为PointXYZ

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	//个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_2;   //创建滤波器对象
	sor_2.setInputCloud(cloud_2);                           //设置待滤波的点云
	sor_2.setMeanK(100);                               //设置在进行统计时考虑查询点临近点数
	sor_2.setStddevMulThresh(0.1);                      //设置判断是否为离群点的阀值
	sor_2.filter(*cloud);                    //存储
	//////////////////////////////////////
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 1cm
	//ne.setRadiusSearch(1);
	ne.setKSearch(20);
	ne.compute(*normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	//采用RANSAC提取平面
	//cout << "Use RANSAC to extract the plane ..." << endl;
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	//pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	//pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients(true);//设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); //设置分割模型
	seg.setNormalDistanceWeight(0.1);//设置表面法线权重系数
	seg.setMethodType(pcl::SAC_RANSAC);//设置采用RANSAC作为算法的参数估计方法
	seg.setMaxIterations(500);             //设置迭代的最大次数
	seg.setDistanceThreshold(0.3);         //设置内点到模型的距离允许最大值
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);
	//std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else {
		writer.write("output.pcd", *cloud_cylinder, false);
	}
	/*********************************************************************/
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_2;
	ne_2.setInputCloud(cloud_cylinder);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZ>());
	ne_2.setSearchMethod(tree_2);
	pcl::PointCloud<pcl::Normal>::Ptr normals_2(new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 1cm
	//ne.setRadiusSearch(1);
	ne_2.setKSearch(20);
	ne_2.compute(*normals_2);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_cylinder, *normals_2, *cloud_with_normals_2);

	////////////////////////////////////////////////////////////////////
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(200);//200
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree_2);
	reg.setNumberOfNeighbours(150);//150
	reg.setInputCloud(cloud_cylinder);
	//reg.setIndices (indices);
	reg.setInputNormals(normals_2);
	reg.setSmoothnessThreshold(7.0 / 180.0 * M_PI);//7
	reg.setCurvatureThreshold(1);//0.1
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	if (clusters.size() < 2) {
		cout << "Not enough clusters !" << endl;
		exit(0);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_3(new pcl::PointCloud<pcl::Normal>);
	for (int i = 0;i < clusters[0].indices.size();i++) {
		// i 是聚类里面的索引
		// clusters[0].indices[i] 可能是原来点云的索引
		ground->push_back(cloud_cylinder->points[clusters[0].indices[i]]);
		normals_3->push_back(normals_2->points[clusters[0].indices[i]]);
	}

	int normal_size = ground->size();
	float ground_normal[3] = { 0 };
	for (int i = 0; i < normal_size; i++) {
		ground_normal[0] += normals_3->points[i].normal[0];
		ground_normal[1] += normals_3->points[i].normal[1];
		ground_normal[2] += normals_3->points[i].normal[2];
	}
	ground_normal[0] /= normal_size;
	ground_normal[1] /= normal_size;
	ground_normal[2] /= normal_size;
	float ground_normal_norm = sqrt(ground_normal[0] * ground_normal[0] + ground_normal[1] * ground_normal[1] + ground_normal[2] * ground_normal[2]);
	ground_normal[0] /= ground_normal_norm;
	ground_normal[1] /= ground_normal_norm;
	ground_normal[2] /= ground_normal_norm;
	cout << "Ground normal vector : " << ground_normal[0] << ", " << ground_normal[1] << ", " << ground_normal[2] << endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pallet(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_4(new pcl::PointCloud<pcl::Normal>);
	for (int i = 0;i < clusters[1].indices.size();i++) {
		pallet->push_back(cloud_cylinder->points[clusters[1].indices[i]]);
		normals_4->push_back(normals_2->points[clusters[1].indices[i]]);
	}
	
	int pallet_normal_size = pallet->size();
	float pallet_normal[3] = { 0 };
	for (int i = 0; i < pallet_normal_size; i++) {
		pallet_normal[0] += normals_4->points[i].normal[0];
		pallet_normal[1] += normals_4->points[i].normal[1];
		pallet_normal[2] += normals_4->points[i].normal[2];
	}
	pallet_normal[0] /= pallet_normal_size;
	pallet_normal[1] /= pallet_normal_size;
	pallet_normal[2] /= pallet_normal_size;
	float pallet_normal_norm = sqrt(pallet_normal[0] * pallet_normal[0] + pallet_normal[1] * pallet_normal[1] + pallet_normal[2] * pallet_normal[2]);
	pallet_normal[0] /= pallet_normal_norm;
	pallet_normal[1] /= pallet_normal_norm;
	pallet_normal[2] /= pallet_normal_norm;
	cout << "Pallet vector : " << pallet_normal[0] << ", " << pallet_normal[1] << ", " << pallet_normal[2] << endl;

	float dot = ground_normal[0] * pallet_normal[0] + ground_normal[1] * pallet_normal[1] + ground_normal[2] * pallet_normal[2];
	cout << "Dot : " << dot << endl;
	float theta = abs(acos(abs(dot)) - M_PI/2);
	cout << "theta: " << theta << " rad = " << theta * 180 / M_PI <<"°"<<endl;

	float special_point[6] = { pallet->points[0].x, pallet->points[0].x, pallet->points[0].y, pallet->points[0].y, pallet->points[0].z, pallet->points[0].z };
	for (int i = 0; i < pallet_normal_size; i++) {
		if (pallet->points[i].x < special_point[0]) special_point[0] = pallet->points[i].x;
		if (pallet->points[i].x > special_point[1]) special_point[1] = pallet->points[i].x;
		if (pallet->points[i].y < special_point[2]) special_point[3] = pallet->points[i].y;
		if (pallet->points[i].y > special_point[3]) special_point[4] = pallet->points[i].y;
		if (pallet->points[i].z < special_point[4]) special_point[5] = pallet->points[i].z;
		if (pallet->points[i].z > special_point[5]) special_point[6] = pallet->points[i].z;
	}
	float center_x = 0.5*(special_point[0]+ special_point[1]);
	float center_y = 0.5*(special_point[2] + special_point[3]);
	float center_z = 0.5*(special_point[4] + special_point[5]);
	cout << "Center: " << center_x << ", "<< center_y << ", " << center_z << endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "color");
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}