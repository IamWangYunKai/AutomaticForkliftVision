#include "stdafx.h"
#include <iostream>
#include <thread>

#define NUM 10000
using namespace std;

float arr[NUM] = { 0 };

void calc(int i) {
	arr[i] = pow(arr[i]/1000.0f, 100);
	return;
}

int main() {
	for (int i = 0; i < NUM; i++) {
		arr[i] = i;
	}
	clock_t start, end;
	start = clock();
	for (int i = 0;i < NUM;i++) {
		std::thread threads{ calc,i };
		threads.join();
	}
	end = clock();
	float dur = (double)(end - start);
	cout << "Use Time: " << dur << endl;
	

	for (int i = 0; i < NUM; i++) {
		arr[i] = i;
	}
	start = clock();
	for (int i = 0;i < NUM;i++) {
		calc(i);
	}
	end = clock();
	dur = (double)(end - start);
	cout << "Use Time: " << dur << endl;
	system("pause");
	return 0;
}