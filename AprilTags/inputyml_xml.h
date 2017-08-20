#include <string>
#include <typeinfo>
#include <iostream>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;
void getCalMat(string &topen,Mat &cameraMatrix,Mat &distortionMatrix){
    FileStorage fs(topen,FileStorage::READ);
    FileNode fn = fs["Camera_Matrix"];
    FileNodeIterator it = fn["data"].begin();
    int i = 0, j = 0;
    for(;it!=fn["data"].end();it++){
        cameraMatrix.at<double>(i,j) = (double)*it;
        ++j;
        if(j == 3){
            j = 0;
            ++i;
        }
    }
    fn = fs["Distortion_Coefficients"];
    i = 0;
    for(it=fn["data"].begin();it!=fn["data"].end();i++,it++)
        distortionMatrix.at<double>(0,i) = (double)*it;
}
