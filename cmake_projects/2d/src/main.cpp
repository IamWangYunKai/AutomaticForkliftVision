#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>
#include <cmath>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/eigen.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/visualization/common/float_image_utils.h>

#include <pcl/io/png_io.h>
//#include <vtkImageImport.h>
//#include <vtkPNGWriter.h>
//#include <vtkSmartPointer.h>
//#include <vtkImageFlip.h>
//#include <pcl/visualization/range_image_visualizer.h>



int main() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//点云对象的读取
	pcl::PCDReader reader;
	reader.read("4.pcd", *input_cloud);
	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
	float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel = 0.00;
	float minRange = 0.0f;
	int borderSize = 1;


	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*input_cloud, angularResolution, maxAngleWidth, maxAngleHeight,
		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

	//②以下保存深度图（保存成png格式）

	float* ranges = rangeImage.getRangesArray();
	unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage.width, rangeImage.height);
	pcl::io::saveRgbPNGFile("saveRangeImageRGB.png", rgb_image, rangeImage.width, rangeImage.height);

	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//点云对象的读取
	pcl::PCDReader reader;
	reader.read("input.pcd", *input_cloud);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	float theta_x = - 20 * M_PI / 180.0f;
	transform.rotate(Eigen::AngleAxisf(theta_x, Eigen::Vector3f::UnitX()));
	float theta_z = -5 * M_PI / 180.0f;
	transform.rotate(Eigen::AngleAxisf(theta_z, Eigen::Vector3f::UnitZ()));
	transform.translation() << 0.0, -0.65, -0.7;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tsf(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*input_cloud, *cloud_tsf, transform);
	

	
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
	*/
	return 0;
}