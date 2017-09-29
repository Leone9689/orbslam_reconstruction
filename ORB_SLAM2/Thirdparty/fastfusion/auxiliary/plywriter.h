#ifndef WRITEPLY_H
#define WRITEPLY_H

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/contrib/contrib.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "ocv_tools.hpp"
#include <cstdlib>
#include <cstdio>
#include <cstring>

bool writePLY(const std::string& filename, const cv::Mat &disparity,cv::Mat img, const cv::Mat &Q, const cv::Mat &rH);
bool writePLYBinary(const std::string& filename, const cv::Mat &disparity,cv::Mat img, const cv::Mat &Q, const cv::Mat &rH);
bool writePLYVerticesOnly(const std::string& filename, const cv::Mat &disparity, const cv::Mat &img, const cv::Mat &Q, const cv::Mat &rH);


#endif // WRITEPLY_H
