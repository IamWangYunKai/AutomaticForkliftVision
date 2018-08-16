#define _CRT_SECURE_NO_DEPRECATE

#include "camera.h"
#include <iostream>
#include <cmath>
#include <time.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/eigen.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define DEBUG false
using namespace std;

int main() {
	Camera my_camera;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    my_camera.setExposureTime(4000);
	my_camera.getData(input_cloud);
	cout << "Reading Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	//根据摄像头外参坐标转换
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	float theta_x = - 20 * M_PI / 180.0f;
	transform.rotate(Eigen::AngleAxisf(theta_x, Eigen::Vector3f::UnitX()));
	float theta_z = -5 * M_PI / 180.0f;
	transform.rotate(Eigen::AngleAxisf(theta_z, Eigen::Vector3f::UnitZ()));
	transform.translation() << 0.0, -0.65, -0.7;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tsf(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*input_cloud, *cloud_tsf, transform);
	cout << "Transform Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;

	//直通滤波器切割点云
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_tsf);            //设置输入点云
	pass.setFilterFieldName("y");         //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(-0.14, -0.03);        //设置在过滤字段的范围		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pass.filter(*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered
	cout << "Pass through Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;

	//切割后可视化
	if (DEBUG) {
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "tsf");
		viewer->addCoordinateSystem();//红色是X轴，绿色是Y轴，蓝色是Z
		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}

	//统计滤波器去噪声
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_2;   //创建滤波器对象
	sor_2.setInputCloud(cloud_filtered);                           //设置待滤波的点云
	sor_2.setMeanK(20);                               //设置在进行统计时考虑查询点临近点数
	sor_2.setStddevMulThresh(0.5);                    //设置判断是否为离群点的阀值
	sor_2.filter(*cloud);                    //存储
	cout << "StatisticalOutlierRemoval Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	
	//法向量估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(20);
	ne.compute(*normals);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	cout << "First Normal Estimation Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;

	//采用RANSAC分割地面
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	seg.setOptimizeCoefficients(true);     //设置对估计的模型系数需要进行优化
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); //设置分割模型
	seg.setNormalDistanceWeight(0.1);      //设置表面法线权重系数
	seg.setMethodType(pcl::SAC_RANSAC);    //设置采用RANSAC作为算法的参数估计方法
	seg.setMaxIterations(100);             //设置迭代的最大次数
	seg.setDistanceThreshold(0.1);         //设置内点到模型的距离允许最大值
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	seg.segment(*inliers_plane, *coefficients_plane);
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_plane);
	extract.setNegative(true);            //保留平面意外的区域
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_cylinder);
	cout << "RANSAC Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;

	//法向量重新估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_2;
	ne_2.setInputCloud(cloud_cylinder);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZ>());
	ne_2.setSearchMethod(tree_2);
	pcl::PointCloud<pcl::Normal>::Ptr normals_2(new pcl::PointCloud<pcl::Normal>);
	ne_2.setKSearch(20);
	ne_2.compute(*normals_2);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_cylinder, *normals_2, *cloud_with_normals_2);
	cout << "Second Normal Estimation Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;

	//区域增长算法找到三个端面
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(300);
	reg.setSearchMethod(tree_2);
	reg.setNumberOfNeighbours(20);
	reg.setSmoothModeFlag(true);
	reg.setCurvatureTestFlag(true);
	reg.setResidualTestFlag(true);
	reg.setInputCloud(cloud_cylinder);
	reg.setInputNormals(normals_2);
	reg.setSmoothnessThreshold(7.0f / 180.0f * M_PI);  //判断法向量夹角
	reg.setCurvatureThreshold(0.1);                    //判断曲率
	reg.setResidualThreshold(0.05);                    //判断投影
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	cout << "Region Growing Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	cout << "Number of clusters is equal to " << clusters.size() << endl;

	pcl::PointCloud<pcl::Normal>::Ptr pallet_avg_normal(new pcl::PointCloud<pcl::Normal>);//三个面的法向量
	pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>());     //三个面的中心坐标
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

		int pallet_normal_size = (int)pallet->size();
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
		pallet_normal[1] = 0; // /= pallet_normal_size;//消除栈板前端面法向量中垂直地面的分量
		pallet_normal[2] /= pallet_normal_size;
		center_point[0] /= pallet_normal_size;
		center_point[1] /= pallet_normal_size;
		center_point[2] /= pallet_normal_size;
		//法向量归一化
		float pallet_normal_norm = sqrt(pallet_normal[0] * pallet_normal[0] + pallet_normal[1] * pallet_normal[1] + pallet_normal[2] * pallet_normal[2]);
		pallet_normal[0] /= pallet_normal_norm;
		pallet_normal[1] /= pallet_normal_norm;
		pallet_normal[2] /= pallet_normal_norm;
		pallet_avg_normal->push_back(pcl::Normal(pallet_normal[0], pallet_normal[1], pallet_normal[2]));
		center->push_back(pcl::PointXYZ(center_point[0], center_point[1], center_point[2]));
	}

	float pallet_normal_final[3] = { 0 };   //最终认为的栈板前表面法向量
	float pallet_center_final[3] = { 0 };   //最终认为的栈板前表面中点
	float total_normal_size = 0;            //所有栈板前表面点的个数
	int total_size = (int)pallet_avg_normal->size();//聚类数量
	for (int i = 0; i < total_size; i++) {
		cout << pallet_avg_normal->points[i] << endl;
		pallet_normal_final[0] += pallet_avg_normal->points[i].normal_x * clusters[i].indices.size();
		pallet_normal_final[1] += pallet_avg_normal->points[i].normal_y * clusters[i].indices.size();
		pallet_normal_final[2] += pallet_avg_normal->points[i].normal_z * clusters[i].indices.size();
		pallet_center_final[0] += center->points[i].x;
		pallet_center_final[1] += center->points[i].y;
		pallet_center_final[2] += center->points[i].z;
		total_normal_size += clusters[i].indices.size();
	}
	pallet_normal_final[0] /= total_normal_size;  //取加权平均
	pallet_normal_final[1] /= total_normal_size;
	pallet_normal_final[2] /= total_normal_size;
	pallet_center_final[0] /= total_size;
	pallet_center_final[1] /= total_size;
	pallet_center_final[2] /= total_size;
	
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
		cout << "Dist to center:";
		cout << sqrt(pow(center->points[i].x - pallet_center_final[0], 2)
			       + pow(center->points[i].y - pallet_center_final[1], 2)
			       + pow(center->points[i].z - pallet_center_final[2], 2)) << endl;
	}

	cout << "Totle Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	
	//可视化
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "color");
	viewer->addCoordinateSystem();//红色是X轴，绿色是Y轴，蓝色是Z
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	return 0;
}
