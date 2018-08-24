#ifndef CAMERA_H
#define CAMERA_H

#include <robokit/core/rbk_core.h>
#include <robokit/core/tf_server/tf/include/tf/transform_listener.h>
#include <robokit/foundation/utils/geometry.h>
#include <robokit/foundation/utils/geo_utils.h>
#include <robokit/foundation/quadtree/quadtree.h>
#include <robokit/algorithm/ray_trace/laser_utils.h>
#include <robokit/chasis/model.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <pmdsdk2.h>

using namespace rbk::protocol;
using namespace rbk;
using namespace rbk::core;

class Camera :public NPluginInterface
{
public:
	Camera();
	~Camera();
	void run();
	std::vector<float> RecogniseTray();
	void loadFromConfigFile();
	void setSubscriberCallBack();

public:
	bool setExposureTime(unsigned int exposure);
	unsigned int getExposureTime();
	bool getPointCloud(std::vector<float> &pointcloud);
	bool getData(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	bool getDoubleData(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	bool getTripleData(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	bool writeData();
	bool writeDoubleData();
	bool writeTripleData();
	void getResult(std::vector<float> &result);
private:
	PMDHandle hnd;
	int res;
	PMDDataDescription dd;
	unsigned int exposureTime;
	std::vector<float> amp;
	std::vector<unsigned> flags;
	int imgHeight;
	int imgWidth;
	char err[256];
};

RBK_INHERIT_PROVIDER(Camera, NPluginInterface, "1.0.0");

#endif