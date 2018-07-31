#include "stdafx.h"
/*
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	cloud->width = 15;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	// Generate the data
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1.0;
	}

	// Set a few outliers
	cloud->points[0].z = 2.0;
	cloud->points[3].z = -2.0;
	cloud->points[6].z = 4.0;

	std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;

	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
	for (size_t i = 0; i < inliers->indices.size(); ++i)
		std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
		<< cloud->points[inliers->indices[i]].y << " "
		<< cloud->points[inliers->indices[i]].z << std::endl;

	return (0);
}
*/
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>("r1.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	int counter = 0;
	while (counter < clusters[0].indices.size())
	{
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	}

	return (0);
}