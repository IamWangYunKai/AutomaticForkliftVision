﻿#include "camera.h"
#include <iostream>
#include <time.h>
#include <string>

#define SOURCE_PLUGIN "O3D3xxCamera.W64.pap"
#define SOURCE_PARAM "192.168.0.70:80:50010"
#define PROC_PLUGIN "O3D3xxProc.W64.ppp"
#define PROC_PARAM ""

using namespace std;

namespace{
	int maxExposureTime = 8000;
	int minExposureTime = 4000;
};

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
}