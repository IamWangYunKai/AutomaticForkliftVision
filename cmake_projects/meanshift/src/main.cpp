#include <iostream>
#include <cmath>
#include <math.h>
#include <time.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/eigen.h>
#include "meanshift.h"

int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cld (new pcl::PointCloud<pcl::PointXYZ>);		
    int val = 10;
    //鏋勯€犵珛鏂逛綋  
    float cube_len = 1;  
    srand(time(NULL));
    for (float x = 0; x < cube_len; x += 0.1)  {  
    	for (float y = 0; y < cube_len; y += 0.1)  {  
    		for (float z = 0; z < cube_len; z += 0.1)  {  
    			pcl::PointXYZ basic_point;    

    			//娌跨潃鍚戦噺(2.5, 2.5, 2.5)骞崇Щ  
    			basic_point.x = (float)(rand() % val) / 10.0f + 2.5;    
    			basic_point.y = (float)(rand() % val) / 10.0f + 2.5;    
    			basic_point.z = (float)(rand() % val) / 10.0f + 2.5;    
    			cld->points.push_back(basic_point);    
    		}  
    	}  
    }  

    for (float x = 0; x < cube_len; x += 0.1)  {  
    	for (float y = 0; y < cube_len; y += 0.1)  {  
    		for (float z = 0; z < cube_len; z += 0.1)  {  
    			pcl::PointXYZ basic_point;    
    			basic_point.x = (float)(rand() % val) / 10.0f - 2.5;    
    			basic_point.y = (float)(rand() % val) / 10.0f - 2.5;    
    			basic_point.z = (float)(rand() % val) / 10.0f - 2.5;    
    			cld->points.push_back(basic_point);    
    		}  
    	}  
    }  

    cld->width = (int)cld->points.size();    
    cld->height = 1;  
    io::savePCDFileASCII("manual_build.pcd", *cld);

    MeanShift ms;
    ms.setInputCloud(cld);
    ms.setKNNRadius(0.7);
    ms.process();
    ms.SaveFile(".", "meanshift");
    system("pause");
}