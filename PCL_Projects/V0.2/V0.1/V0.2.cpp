#include "stdafx.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <time.h>
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
	cout << "Reading Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	//statistical Outlier Removal
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*input_cloud, *cloud_2);//把PCLPointCloud2转换为PointXYZ
	//pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_2);//把PCLPointCloud2转换为PointXYZ

	// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
	//个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_2;   //创建滤波器对象
	sor_2.setInputCloud(cloud_2);                           //设置待滤波的点云
	sor_2.setMeanK(20);                               //设置在进行统计时考虑查询点临近点数
	sor_2.setStddevMulThresh(0.5);                    //设置判断是否为离群点的阀值
	sor_2.filter(*cloud);                    //存储
	cout << "StatisticalOutlierRemoval Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
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
	cout << "First Normal Estimation Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
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

	// Extract the planar inliers from the input cloud
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_cylinder);
	/*
	if (cloud_cylinder->points.empty())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else {
		writer.write("output.pcd", *cloud_cylinder, false);
	}
	*/
	cout << "RANSAC Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	/////////////////////////////////////////////////
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

	cout << "Second Normal Estimation Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	////////////////////////////////////////////////////////////////////
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);//200
	reg.setMaxClusterSize(500);
	reg.setSearchMethod(tree_2);
	reg.setNumberOfNeighbours(20);//150
	reg.setSmoothModeFlag(true);
	reg.setCurvatureTestFlag(true);
	reg.setResidualTestFlag(true);
	reg.setInputCloud(cloud_cylinder);
	reg.setInputNormals(normals_2);
	reg.setSmoothnessThreshold(7.0f / 180.0f * M_PI);  //判断法向量夹角
	reg.setCurvatureThreshold(0.1);                    //判断曲率
	reg.setResidualThreshold(0.05);                     //判断投影

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	cout << "Region Growing Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;

	pcl::PointCloud<pcl::Normal>::Ptr pallet_avg_normal(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>());
	if (clusters.size() < 2) {
		cout << "No enough clusters !" << endl;
		exit(0);
	}
	for (int num = 0; num < clusters.size(); num++) {
		cout << "cluster " << num << ": " << clusters[num].indices.size() << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pallet(new pcl::PointCloud<pcl::PointXYZ>);
		pallet->clear();
		pcl::PointCloud<pcl::Normal>::Ptr normals_4(new pcl::PointCloud<pcl::Normal>);
		normals_4->clear();
		for (int i = 0;i < clusters[num].indices.size();i++) {
			pallet->push_back(cloud_cylinder->points[clusters[num].indices[i]]);
			normals_4->push_back(normals_2->points[clusters[num].indices[i]]);
		}

		int pallet_normal_size = pallet->size();
		float pallet_normal[3] = { 0 };
		float center_point[3] = { 0 };
		for (int i = 0; i < pallet_normal_size; i++) {
			pallet_normal[0] += normals_4->points[i].normal[0];
			pallet_normal[1] += normals_4->points[i].normal[1];
			pallet_normal[2] += normals_4->points[i].normal[2];
			center_point[0] += pallet->points[i].x;
			center_point[1] += pallet->points[i].y;
			center_point[2] += pallet->points[i].z;
		}
		pallet_normal[0] /= pallet_normal_size;
		pallet_normal[1] /= pallet_normal_size;
		pallet_normal[2] /= pallet_normal_size;
		center_point[0] /= pallet_normal_size;
		center_point[1] /= pallet_normal_size;
		center_point[2] /= pallet_normal_size;

		float pallet_normal_norm = sqrt(pallet_normal[0] * pallet_normal[0] + pallet_normal[1] * pallet_normal[1] + pallet_normal[2] * pallet_normal[2]);
		pallet_normal[0] /= pallet_normal_norm;
		pallet_normal[1] /= pallet_normal_norm;
		pallet_normal[2] /= pallet_normal_norm;
		pallet_avg_normal->push_back(pcl::Normal(pallet_normal[0], pallet_normal[1], pallet_normal[2]));
		center->push_back(pcl::PointXYZ(center_point[0], center_point[1], center_point[2]));
	}

	float pallet_normal_final[3] = { 0 };//最终认为的栈板前表面法向量
	float total_normal_size = 0;//所有栈板前表面点的个数
	int total_size = pallet_avg_normal->size();//聚类数量
	for (int i = 0; i < total_size; i++) {
		cout << pallet_avg_normal->points[i] << endl;
		pallet_normal_final[0] += pallet_avg_normal->points[i].normal_x * clusters[i].indices.size();
		pallet_normal_final[1] += pallet_avg_normal->points[i].normal_y * clusters[i].indices.size();
		pallet_normal_final[2] += pallet_avg_normal->points[i].normal_z * clusters[i].indices.size();
		total_normal_size += clusters[i].indices.size();
	}
	pallet_normal_final[0] /= total_normal_size;//取加权平均
	pallet_normal_final[1] /= total_normal_size;
	pallet_normal_final[2] /= total_normal_size;
	
	for (int i = 0; i < total_size; i++) {
		cout << "Delta theta : ";
		cout << acos(pallet_avg_normal->points[i].normal_x*pallet_normal_final[0]
			+ pallet_avg_normal->points[i].normal_y*pallet_normal_final[1]
			+ pallet_avg_normal->points[i].normal_z*pallet_normal_final[2]) * 180 / M_PI << "°" << endl;
	}
	for (int i = 0; i < total_size; i++) {
		cout << "Dist : ";
		cout << sqrt( pow(center->points[i].x - center->points[(i + 1) % total_size].x, 2)
					+ pow(center->points[i].y - center->points[(i + 1) % total_size].y, 2)
					+ pow(center->points[i].z - center->points[(i + 1) % total_size].z, 2) )<< endl;
	}

	cout << "Totle Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
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