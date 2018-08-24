#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  

using namespace cv;

int main() {  
    Mat img = imread("avatar.jpg");
    // 鍦ㄧ獥鍙ｄ腑鏄剧ずavatar  
    imshow("avatar", img);
    // 绛夊緟6000 ms鍚庣獥鍙ｈ嚜鍔ㄥ叧闂?   
    waitKey(6000);
}