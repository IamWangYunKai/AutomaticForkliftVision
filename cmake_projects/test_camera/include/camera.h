#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <vector>
#include <pmdsdk2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class Camera {
public:
	Camera();
	~Camera();
	bool setExposureTime(unsigned int exposure);
	unsigned int getExposureTime();
	bool getData(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	bool writeData();
private:
	PMDHandle hnd;
	int res;
	PMDDataDescription dd;
	unsigned int exposureTime;
	//std::vector<float> amp;
	//std::vector<unsigned> flags;
	int imgHeight;
	int imgWidth;
	char err[256];
}

#endif