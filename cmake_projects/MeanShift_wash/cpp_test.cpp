#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "MeanShift.h"

using namespace std;
namespace{
    int max_points_num = 50;
    double max_dist = 6.0;
    double kernel_bandwidth = 0.1;
};

vector<vector<double> > load_points(const char *filename) {
    vector<vector<double> > points;
    FILE *fp = fopen(filename, "r");
    char line[50];
    while (fgets(line, sizeof(line), fp) != NULL) {
        double x, y;
        char *x_str = line;
        char *y_str = line;
        while (*y_str != '\0') {
            if (*y_str == ',') {
                *y_str++ = 0;
                x = atof(x_str);
                y = atof(y_str);
                vector<double> point;
                point.push_back(x);
                point.push_back(y);
                points.push_back(point);
                break;
            }
            ++y_str;
        }
    }
    fclose(fp);
    return points;
}

void print_points(vector<vector<double> > points){
    for(int i=0; i<points.size(); i++){
        for(int dim = 0; dim<points[i].size(); dim++) {
            printf("%f ", points[i][dim]);
        }
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    MeanShift *msp = new MeanShift();
    vector<vector<double> > points = load_points("test.csv");

    //wash the data
    vector<vector<double> > washed_points;
    for(int i = 0; i < points.size(); i ++){
        double dist = sqrt(points[i][0]*points[i][0] + points[i][1]*points[i][1]);
        if(dist > max_dist) continue;
        else{
            washed_points.push_back(points[i]);
        }
    }
    printf("Number of washed data : %d\n", washed_points.size());

    vector<Cluster> clusters = msp->cluster(washed_points, kernel_bandwidth);

    if(clusters.size() < 4){
        printf("!!! No enough cluster !!!\n");
        exit(0);
    }

    vector<vector<double> > center;
    for(int i = 0; i < clusters.size(); i++){
        double x = 0;
        double y = 0;
        vector<double> center_point;
        int point_num = clusters[i].original_points.size();
        for(int j = 0; j < point_num; j++){
            x += clusters[i].original_points[j][0];
            y += clusters[i].original_points[j][1];
        }
        x /= point_num;
        y /= point_num;
        center_point.push_back(x);
        center_point.push_back(y);
        center.push_back(center_point);
        printf("center: %d : ( %f, %f )\n", i, x, y);
    }

    vector<double> center_dist;
    for(int i = 0; i < center.size(); i++){
        for(int j = i + 1; j < center.size(); j++){
            double dist = sqrt((center[i][0] - center[j][0])*(center[i][0] - center[j][0]) 
                + (center[i][1] - center[j][1])*(center[i][1] - center[j][1]));
            center_dist.push_back(dist);
            printf("center %d to center %d distance : %f\n", i, j, dist);
        }
    }
    printf("Using Time : %f s", (double)clock() / CLOCKS_PER_SEC);
    return 0;
}
