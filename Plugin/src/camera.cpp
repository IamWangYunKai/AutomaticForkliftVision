#include "camera.h"
#include <time.h>
#include <string>
#include <math.h>
#include <robokit/core/time_server/rbk_time.h>
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
#include "message_pointcloud.pb.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#define SOURCE_PLUGIN "O3D3xxCamera.W32.pap"
#define SOURCE_PARAM "192.168.0.70:80:50010"	//摄像头IP地址
#define PROC_PLUGIN "O3D3xxProc.W32.ppp"
#define PROC_PARAM ""

namespace {
	const bool SEE_RESULT = false;
	const float minRange = 1.2;								//最近能看到的栈板距离 （m）
	const float maxRange = 3.2;								//最远能看到的栈板距离 （m）
	const float pitchAngle = -6.5 * M_PI / 180.0f;			//摄像头俯仰角 （°）
	const int maxExposureTime = 8000;						//最大的曝光时间 （μs）
	const int minExposureTime = 4000;
	const float height = -0.90;								//摄像头距离地面的高度 （m）
	const float smoothnessThreshold = 12.0f * M_PI / 180.0f;//区域增长的结束条件 （°）
};

using namespace std;
using namespace rbk::protocol;

RBK_INHERIT_SOURCE(Camera)
Camera::Camera() {
	//connect to the camera
	res = pmdOpen(&hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM);
	if (res != PMD_OK) {
		cout << "Could not connect" << endl;
	}
	setExposureTime(maxExposureTime);
	//start to sample
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
	}

	res = pmdGetSourceDataDescription(hnd, &dd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get data description : " << err << endl;
		pmdClose(hnd);
	}

	imgWidth = dd.img.numColumns;
	imgHeight = dd.img.numRows;
	amp.resize(imgWidth * imgHeight);
	flags.resize(imgWidth * imgHeight);
}

Camera::~Camera() {
	res = pmdClose(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not close the connection : " << err << endl;
	}
}

bool Camera::setExposureTime(unsigned int exposure) {
	exposureTime = exposure;
	res = pmdSetIntegrationTime(hnd, 0, exposure);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not set the integration time : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	return true;
}

unsigned int Camera::getExposureTime() {
	unsigned int exposure = 0;
	res = pmdGetIntegrationTime(hnd, &exposure, 0);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not set the integration time : " << err << endl;
		pmdClose(hnd);
		return -1;
	}
	return exposure;
}

bool Camera::getPointCloud(vector<float> &pointcloud) {
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get amplitude data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}

	pointcloud.clear();
	pointcloud.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &pointcloud[0], pointcloud.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	return true;
}

bool Camera::getData(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
	//start to sample
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get amplitude data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}

	vector<float> xyz3Dcoordinate;
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	input_cloud->clear();
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	return true;
}

bool Camera::getDoubleData(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
	//start to sample
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get amplitude data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}

	vector<float> xyz3Dcoordinate;
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	input_cloud->clear();
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	///////////////////////////////////////////
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get amplitude data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	xyz3Dcoordinate.clear();
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	return true;
}

bool Camera::getTripleData(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
	//start to sample
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get amplitude data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}

	vector<float> xyz3Dcoordinate;
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	input_cloud->clear();
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	///////////////////////////////////////////
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get amplitude data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	xyz3Dcoordinate.clear();
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	///////////////////////////
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get amplitude data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	xyz3Dcoordinate.clear();
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
}

bool Camera::writeData() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//start to sample
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	vector<float> xyz3Dcoordinate;
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	pcl::PCDWriter writer;
	time_t t = time(NULL);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S.pcd", localtime(&t));
	writer.write(string(tmp), *input_cloud, false);
	return true;
}

bool Camera::writeDoubleData() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//start to sample
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	vector<float> xyz3Dcoordinate;
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	///////////////////////////////////////////
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	xyz3Dcoordinate.clear();
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}

	pcl::PCDWriter writer;
	time_t t = time(NULL);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S.pcd", localtime(&t));
	writer.write(string(tmp), *input_cloud, false);
	return true;
}

bool Camera::writeTripleData() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//start to sample
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	vector<float> xyz3Dcoordinate;
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	///////////////////////////////////////////
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	xyz3Dcoordinate.clear();
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	//////////////////////
	res = pmdUpdate(hnd);
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 256);
		cout << "Could not updateData : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get flag data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	xyz3Dcoordinate.clear();
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3);
	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK) {
		pmdGetLastError(hnd, err, 128);
		cout << "Could not get xyz data : " << err << endl;
		pmdClose(hnd);
		return false;
	}
	for (int i = 0; i < flags.size(); i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			input_cloud->push_back(pcl::PointXYZ(xyz3Dcoordinate[i * 3 + 0],
				xyz3Dcoordinate[i * 3 + 1],
				xyz3Dcoordinate[i * 3 + 2]));
		}
	}
	pcl::PCDWriter writer;
	time_t t = time(NULL);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y-%m-%d-%H-%M-%S.pcd", localtime(&t));
	writer.write(string(tmp), *input_cloud, false);
	return true;
}

void Camera::getResult(vector<float> &result) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	getDoubleData(input_cloud);
	cout << "Reading Using Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
	//根据摄像头外参坐标转换
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(pitchAngle, Eigen::Vector3f::UnitX()));
	//float theta_z = -5 * M_PI / 180.0f;
	//transform.rotate(Eigen::AngleAxisf(theta_z, Eigen::Vector3f::UnitZ()));
	transform.translation() << 0.0, height, 0.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tsf(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*input_cloud, *cloud_tsf, transform);

	//直通滤波器切割点云
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_tsf);            //设置输入点云
	pass.setFilterFieldName("y");         //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(-0.15, -0.03);        //设置在过滤字段的范围	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	pass.filter(*cloud_filtered2);            //执行滤波，保存过滤结果在cloud_filtered

	pcl::PassThrough<pcl::PointXYZ> pass2;
	pass2.setInputCloud(cloud_filtered2);            //设置输入点云
	pass2.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
	pass2.setFilterLimits(minRange, maxRange);        //设置在过滤字段的范围	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pass2.filter(*cloud_filtered);            //执行滤波，保存过滤结果在cloud_filtered          //执行滤波，保存过滤结果在cloud_filtered

	//统计滤波器去噪声
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
	sor.setInputCloud(cloud_filtered);                           //设置待滤波的点云
	sor.setMeanK(20);                               //设置在进行统计时考虑查询点临近点数
	sor.setStddevMulThresh(0.5);                    //设置判断是否为离群点的阀值
	sor.filter(*cloud);                    //存储

	//法向量估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(20);
	ne.compute(*normals);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

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
	extract.setNegative(true);            //保留平面以外的区域
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>());
	extract.filter(*cloud_ransac);

	//法向量重新估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_2;
	ne_2.setInputCloud(cloud_ransac);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::search::KdTree<pcl::PointXYZ>());
	ne_2.setSearchMethod(tree_2);
	pcl::PointCloud<pcl::Normal>::Ptr normals_2(new pcl::PointCloud<pcl::Normal>);
	ne_2.setKSearch(20);
	ne_2.compute(*normals_2);
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_2(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::concatenateFields(*cloud_ransac, *normals_2, *cloud_with_normals_2);

	//区域增长算法找到三个端面
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(300);
	reg.setSearchMethod(tree_2);
	reg.setNumberOfNeighbours(20);
	reg.setSmoothModeFlag(true);
	reg.setCurvatureTestFlag(true);
	//reg.setResidualTestFlag(true);
	reg.setInputCloud(cloud_ransac);
	reg.setInputNormals(normals_2);
	reg.setSmoothnessThreshold(smoothnessThreshold);  //判断法向量夹角
	reg.setCurvatureThreshold(1);                    //判断曲率
	//reg.setResidualThreshold(0.05);                    //判断投影
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
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
			pallet->push_back(cloud_ransac->points[clusters[num].indices[i]]);
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
	//转换为四元数, y方向为0，认为只绕y轴转过一定角度
	float theta = atan2(pallet_normal_final[2], pallet_normal_final[0]);
	Eigen::AngleAxisd rotation_vector(theta, Eigen::Vector3d(0, 1, 0));
	Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);

	result.clear();
	result.push_back(pallet_center_final[0]);
	result.push_back(pallet_center_final[1]);
	result.push_back(pallet_center_final[2]);
	result.push_back(q.x());
	result.push_back(q.y());
	result.push_back(q.z());
	result.push_back(q.w());

	for (int i = 0; i < total_size; i++) {
		cout << "Delta theta : ";
		cout << acos(pallet_avg_normal->points[i].normal_x*pallet_normal_final[0]
			+ pallet_avg_normal->points[i].normal_y*pallet_normal_final[1]
			+ pallet_avg_normal->points[i].normal_z*pallet_normal_final[2]) * 180 / M_PI << "°" << endl;
	}
	for (int i = 0; i < total_size; i++) {
		cout << "Dist : ";
		cout << sqrt(pow(center->points[i].x - center->points[(i + 1) % total_size].x, 2)
			+ pow(center->points[i].y - center->points[(i + 1) % total_size].y, 2)
			+ pow(center->points[i].z - center->points[(i + 1) % total_size].z, 2)) << endl;
		cout << "Dist to center:";
		cout << sqrt(pow(center->points[i].x - pallet_center_final[0], 2)
			+ pow(center->points[i].y - pallet_center_final[1], 2)
			+ pow(center->points[i].z - pallet_center_final[2], 2)) << endl;
	}

	cout << "Totle Time : " << (double)clock() / CLOCKS_PER_SEC << "s" << endl;

	//可视化
	if (SEE_RESULT) {
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
		viewer->setBackgroundColor(0, 0, 0);
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
		viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "color");
		viewer->addCoordinateSystem();//红色是X轴，绿色是Y轴，蓝色是Z
		while (!viewer->wasStopped()) {
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}
}

////////////////////////////////////////
void Camera::loadFromConfigFile(){

}

void Camera::setSubscriberCallBack(){
	setServiceCallBack<std::vector<float>()>("RecogniseTray", &Camera::RecogniseTray, this);
}

//死循环
void Camera::run(){
	//do something
	while (true) { 
		rbk::protocol::Message_3DPointCloud pointcloud;
		vector<float> input_cloud;
		getPointCloud(input_cloud);
		for (int i = 0; i < input_cloud.size()/3; i++) {
			pointcloud.add_point();
			rbk::protocol::Message_O3D3o3Point* p = pointcloud.add_point();
			p->set_x(input_cloud[3 * i + 0]);
			p->set_y(input_cloud[3 * i + 1]);
			p->set_z(input_cloud[3 * i + 2]);
			rbk::core::Time t = rbk::core::Time::now();
			pointcloud.mutable_header()->set_data_nsec(t.toNSec());
			pointcloud.mutable_header()->set_frame_id("/O3D303");
		}
		publishTopic(pointcloud);
		SLEEP(500); 
	}
}
//服务
vector<float> Camera::RecogniseTray(){
	vector<float> result;
	// 获取数据 dx dy dz & 四元数(x, y, z, w) 
	getResult(result);
	return result;
}