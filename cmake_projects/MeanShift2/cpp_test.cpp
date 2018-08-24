#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MeanShift.h"

using namespace std;
namespace{
    int max_points_num = 50;
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
    double kernel_bandwidth = 1;

    vector<vector<double> > points = load_points("test.csv");
    vector<Cluster> clusters = msp->cluster(points, kernel_bandwidth);

    FILE *fp = fopen("result.csv", "w");
    if(!fp){
        perror("Couldn't write result.csv");
        exit(0);
    }
    /*
    printf("\n====================\n");
    printf("Found %lu clusters\n", clusters.size());
    printf("====================\n\n");
    for(int cluster = 0; cluster < clusters.size(); cluster++) {
      printf("Cluster %i:\n", cluster);
      for(int point = 0; point < clusters[cluster].original_points.size(); point++){
        for(int dim = 0; dim < clusters[cluster].original_points[point].size(); dim++) {
          printf("%f ", clusters[cluster].original_points[point][dim]);
          fprintf(fp, dim?",%f":"%f", clusters[cluster].original_points[point][dim]);
        }
        printf(" -> ");
        for(int dim = 0; dim < clusters[cluster].shifted_points[point].size(); dim++) {
          printf("%f ", clusters[cluster].shifted_points[point][dim]);
        }
        printf("\n");
        fprintf(fp, "\n");
      }
      printf("\n");
    }
    fclose(fp);
    */
    if(clusters.size() > 1){
        for(int point = 0; point < clusters[1].original_points.size(); point++){
            printf("%f, %f\n", clusters[1].original_points[point][0],clusters[1].original_points[point][1] );
        }
    }
    double kernel_bandwidth_2 = 0.1;
    vector<Cluster> clusters_2 = msp->cluster(clusters[1].original_points, kernel_bandwidth_2);
    for(int i = 0; i < clusters_2.size(); i++) {
        printf("Cluster %d : %d\n", i, clusters_2[i].original_points.size());
    }



    //get each small cluster from each big cluster
    vector<Cluster> multi_cluster;
    for(int i = 0; i < clusters.size(); i++){
        printf("Big Cluster: %d\n", i);
        if(clusters[i].original_points.size() > max_points_num) continue;//not want any wall
        else{
            double kernel_bandwidth_2 = 0.1;
            vector<Cluster> clusters_2 = msp->cluster(clusters[i].original_points, kernel_bandwidth_2);
            for(int j = 0; j < clusters_2.size(); j++) {
                printf("Small Cluster %d : %d points\n", j, clusters_2[j].original_points.size());
                multi_cluster.push_back(clusters_2[j]);
            }
        }
    }

    vector<vector<double> > center;
    for(int i = 0; i < multi_cluster.size(); i++){
        double x = 0;
        double y = 0;
        vector<double> center_point;
        int point_num = multi_cluster[i].original_points.size();
        for(int j = 0; j < point_num; j++){
            x += multi_cluster[i].original_points[j][0];
            y += multi_cluster[i].original_points[j][1];
        }
        x /= point_num;
        y /= point_num;
        center_point.push_back(x);
        center_point.push_back(y);
        center.push_back(center_point);
        printf("center: %d : %f, %f\n", i, x, y);
    }

    vector<double> center_dist;
    for(int i = 0; i < center.size(); i++){
        for(int j = i; j < center.size(); j++){
            double dist = sqrt((center[i][0] - center[j][0])*(center[i][0] - center[j][0]) 
                + (center[i][1] - center[j][1])*(center[i][1] - center[j][1]));
            center_dist.push_back(dist);
            printf("dist : %f\n", dist);
        }
    }
    return 0;
}
