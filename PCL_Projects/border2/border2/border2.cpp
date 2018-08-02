#include "stdafx.h"
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
#include <vector>
#include <boost/thread/thread.hpp>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/visualization/cloud_viewer.h>
#include <cmath>

#include <pcl/surface/gp3.h>
#include <pcl/segmentation/region_growing.h>

#include <gl/gl.h>
#include <gl/glu.h>
#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

int main(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//点云对象的读取
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	reader.read("test.pcd", *cloud);    //读取点云到cloud中
	//首先计算点云法向量
	cout << "Calculate normal vector ..." << endl;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 1cm
	//ne.setRadiusSearch(1);
	ne.setKSearch(20);
	cout << "Computing ..." << endl;
	ne.compute(*normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	cout << "Calculate normal vector Over !" << endl;
	
	//采用RANSAC提取平面
	cout << "Use RANSAC to extract the plane ..." << endl;
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
	seg.setDistanceThreshold(0.2);         //设置内点到模型的距离允许最大值
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	// Obtain the plane inliers and coefficients
	seg.segment(*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);
	cout << "Extract over !" << endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else{
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
		writer.write("output.pcd", *cloud_cylinder, false);
	}
	/*********************************************************************/
	cout << "Calculate normal vector ..." << endl;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_2;
	ne_2.setInputCloud(cloud_cylinder);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZ>());
	ne_2.setSearchMethod(tree_2);
	pcl::PointCloud<pcl::Normal>::Ptr normals_2(new pcl::PointCloud<pcl::Normal>);
	// Use all neighbors in a sphere of radius 1cm
	//ne.setRadiusSearch(1);
	ne_2.setKSearch(20);
	cout << "Computing ..." << endl;
	ne_2.compute(*normals_2);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_cylinder, *normals_2, *cloud_with_normals_2);
	cout << "Calculate normal vector Over !" << endl;
	pcl::io::savePCDFileASCII("output_2.pcd", *cloud_cylinder);

	////////////////////////////////////////////////////////////////////
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud_cylinder);
	//reg.setIndices (indices);
	reg.setInputNormals(normals_2);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	// 可视化聚类的结果
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	}





	/*
	int normal_size = cloud_with_normals_2->size();
	//计算平均的法向量
	float mean_normal[3] = {0};
	for (int i = 0; i < normal_size; i++) {
		mean_normal[0] += cloud_with_normals_2->points[i].normal[0];
		mean_normal[1] += cloud_with_normals_2->points[i].normal[1];
		mean_normal[2] += cloud_with_normals_2->points[i].normal[2];
	}
	mean_normal[0] /= normal_size;
	mean_normal[1] /= normal_size;
	mean_normal[2] /= normal_size;
	float a_norm = sqrt(mean_normal[0] * mean_normal[0] 
		+ mean_normal[1] * mean_normal[1] 
		+ mean_normal[2] * mean_normal[2]);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr normals_new(new pcl::PointCloud<pcl::Normal>);
	for (int i = 0; i < normal_size; i++) {
		float dot = mean_normal[0] * cloud_with_normals_2->points[i].normal[0]
			+ mean_normal[1] * cloud_with_normals_2->points[i].normal[1]
			+ mean_normal[2] * cloud_with_normals_2->points[i].normal[2];
		float b_norm = sqrt(cloud_with_normals_2->points[i].normal[0]* cloud_with_normals_2->points[i].normal[0]
			+ cloud_with_normals_2->points[i].normal[1]* cloud_with_normals_2->points[i].normal[1]
			+ cloud_with_normals_2->points[i].normal[2]* cloud_with_normals_2->points[i].normal[2]);
		float theta = dot / (a_norm * b_norm);
		if (fabs(theta) < M_PI/18) {
			cloud_new->push_back(cloud_cylinder->points[i]);
			normals_new->push_back(normals_2->points[i]);
		}
	}
	*/

	////////////////////
	/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_new, "cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/


	/*
	//显示法向量
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_new, "cloud");

	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_new, normals_new, 3, 0.03, "normals");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/
	///////////////////////////////////////////////////////////////////

	/*******************************************************************/
	/*
	//calculate boundary;
	pcl::PointCloud<pcl::Boundary> boundary;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	est.setInputCloud(cloud_cylinder);
	est.setInputNormals(normals_2);
	est.setSearchMethod(tree_2);
	est.setKSearch(50); //一般这里的数值越高，最终边界识别的精度越好
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	est.compute(boundary);

	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i<cloud_cylinder->size(); i++) {
		uint8_t x = (boundary.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1){
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloud_cylinder->points[i]);
			countBoundaries++;
		}
		else noBoundPoints.push_back(cloud_cylinder->points[i]);
	}
	std::cout << "boudary size is：" << countBoundaries << std::endl;
	//  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);
	pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
	pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);


	//pcl::visualization::CloudViewer viewer("test");
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Vista 3D"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(boundPoints, "cloud");
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
	*/
}