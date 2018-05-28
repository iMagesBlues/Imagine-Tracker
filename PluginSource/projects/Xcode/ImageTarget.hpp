//
//  ImageTarget.hpp
//  libimaginear
//
//  Created by Ronnie Miguel Besas on 5/13/18.
//  Copyright © 2018 Ronnie Miguel Besas. All rights reserved.
//

#ifndef ImageTarget_hpp
#define ImageTarget_hpp

#include <stdio.h>
//#include <string>
#include "opencv2/opencv.hpp"



using namespace cv;
using namespace std;

struct ImageTarget
{
    string           name;
    Size             size;
    Mat              image;
    Mat              grayImg;
    
    vector<KeyPoint> keypoints;
    Mat              descriptors;
    
    vector<Point2f>  points2d;
    vector<Point3f>  points3d;
    
    int BuildFromImage(const Mat& img, string imgName);
    void GetGray(const Mat& image, Mat& gray);
    void ExportDatabase();
    void ImportDatabase(string imgName);
    
};

/**
 * Intermediate pattern tracking info structure
 */
struct TrackingInfo
{
    cv::Mat                   homography;
    std::vector<cv::Point2f>  points2d;
    
    void draw2dContour(cv::Mat& image, cv::Scalar color) const;
};

#endif /* ImageTarget_hpp */
