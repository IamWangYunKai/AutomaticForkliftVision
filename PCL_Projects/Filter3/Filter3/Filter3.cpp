#include "stdafx.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int main(int argc, char** argv) {
	if (argc != 2) {  //确保输入的参数
		std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
		exit(0);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	//点云对象的读取
	pcl::PCDReader reader;
	reader.read("test.pcd", *cloud);    //读取点云到cloud中

	if (strcmp(argv[1], "-r") == 0) {
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建滤波器

		outrem.setInputCloud(cloud);    //设置输入点云
		outrem.setRadiusSearch(0.2);     //设置半径为0.8的范围内找临近点
		outrem.setMinNeighborsInRadius(30); //设置查询点的邻域点集数小于2的删除
										   // apply filter
		outrem.filter(*cloud_filtered);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ>("output_1.pcd", *cloud_filtered, false);
	}
	else if (strcmp(argv[1], "-c") == 0) {
		//创建条件限定的下的滤波器
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new
			pcl::ConditionAnd<pcl::PointXYZ>());   //创建条件定义对象
												   //为条件定义对象添加比较算子
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 1.1)));   //添加在Z字段上大于0的比较算子

		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
			pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 2.5)));   //添加在Z字段上小于0.8的比较算子
																					   // 创建滤波器并用条件定义对象初始化
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setCondition(range_cond);
		condrem.setInputCloud(cloud);                   //输入点云
		condrem.setKeepOrganized(true);               //设置保持点云的结构
													  // 执行滤波
		condrem.filter(*cloud_filtered);  //大于0.0小于0.8这两个条件用于建立滤波器
		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ>("output_2.pcd", *cloud_filtered, false);
	}
	else {
		std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
		exit(0);
	}
	return (0);
}