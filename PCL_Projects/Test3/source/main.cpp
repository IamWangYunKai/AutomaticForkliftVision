//#include "stdafx.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>//标准C++库中的输入输出类相关头文件。
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer){
	viewer.setBackgroundColor(1.0, 1.0, 1.0);   //设置背景颜色
}

int main(){
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::loadPCDFile("test.pcd", *cloud);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //创建viewer对象
	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");
	return 0;
}