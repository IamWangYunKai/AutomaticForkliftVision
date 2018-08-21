#include <iostream>
#include <vector>
#include "hello.h"

using namespace std;

struct Point {
	float x;
	float y;
	float z;
};

void getData(vector<Point> &input){
	input.push_back(Point{1, 2, 3});
	input.push_back(Point{4, 5, 6});
}

int main() {
	hello();
	//vector<Point> point_cloud;
	vector<Point> point_cloud;
	getData(point_cloud);
	cout << point_cloud[0].x << " , " << point_cloud[1].z << endl;
	//cout << point_cloud.x << point_cloud.y << point_cloud.z <<endl;
	system("pause");
	return 0;
}