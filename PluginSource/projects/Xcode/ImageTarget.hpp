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
#include "CameraCalibration.hpp"
#include "GeometryTypes.hpp"


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
    void ExportDatabase();
    void ImportDatabase(string imgName);
};

/**
 * Intermediate pattern tracking info structure
 */
struct TrackingInfo
{
    
    bool                      found = false;
    cv::Mat                   raw_homography;
    cv::Mat                   maskROI;
    std::vector<cv::Point2f>  raw_points2d;
    Transformation            raw_pose3d;
    
    cv::KalmanFilter          kf;
    cv::Mat                   kf_state;
    cv::Mat                   kf_meas;
    bool                      kf_has_prediction;

    ImageTarget               kf_imagetarget;
    cv::Mat                   kf_homography;
    std::vector<cv::Point2f>  kf_projectedpoints;
    cv::Mat                   kf_tvec = Mat_<float>(3,1);
    cv::Mat                   kf_rvec = Mat_<float>(3,1);
    Transformation            kf_pose3d;
    float                     steadystate = 0;
    float                     kf_lostTime = 0.25f;
    
    cv::Mat                   finalHomography;
    Transformation            finalPose3D;
    
    CameraCalibration   calib;
    
    void drawRawOutline(cv::Mat& image, cv::Scalar color) const;
    void computeRawPose(const ImageTarget& imageTarget, const CameraCalibration& calibration);
    void showAxes(CameraCalibration calib, Transformation tMat, Mat& img);
    void initKalman(const ImageTarget& imageTarget, const CameraCalibration& calibration);
    void updateKalman();
    void updateKalman(const double dT);

    void predictKalman();
    void predictKalman(const double dT);

    void correctKalman();
    void resetKalman();
    void drawKalmanOutline(cv::Mat& img);
    
    void finalizeHomography();
    void drawFinalOutline(cv::Mat& img);
    
    double kf_lastTick;
    double kf_tick;
};


#endif /* ImageTarget_hpp */
