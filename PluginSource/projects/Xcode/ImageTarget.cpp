//
//  ImageTarget.cpp
//  libimaginear
//
//  Created by Ronnie Miguel Besas on 5/13/18.
//  Copyright © 2018 Ronnie Miguel Besas. All rights reserved.
//

#include "ImageTarget.hpp"
#include "DebugCPP.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "CameraCalibration.hpp"
#include "GeometryTypes.hpp"
#include "ARUtils.hpp"

int ImageTarget::BuildFromImage(const Mat& scaledImage, string imgName){
    bool result = true;
    
    name = imgName;
    image = scaledImage.clone();
    size = Size(image.cols, image.rows);
    ARUtils::GetGraySharp(image, grayImg);
    
    points2d.resize(4);
    points3d.resize(4);
    
    // Image dimensions
    const float w = image.cols;
    const float h = image.rows;
    
    Debug::Log("img w and h", Color::Blue);
    Debug::Log(w);
    Debug::Log(h);
    
    // Normalized dimensions:
    const float maxSize = std::max(w,h);
    const float unitW = w / maxSize;
    const float unitH = h / maxSize;
    
    points2d[0] = cv::Point2f(0,0);
    points2d[1] = cv::Point2f(w,0);
    points2d[2] = cv::Point2f(w,h);
    points2d[3] = cv::Point2f(0,h);
    
    points3d[0] = cv::Point3f(-unitW, -unitH, 0);
    points3d[1] = cv::Point3f( unitW, -unitH, 0);
    points3d[2] = cv::Point3f( unitW,  unitH, 0);
    points3d[3] = cv::Point3f(-unitW,  unitH, 0);
    
    //detect keypoints
    Ptr<FeatureDetector> detector  = ORB::create(500);
    detector->detectAndCompute(grayImg, noArray(), keypoints, descriptors);
    
    if(keypoints.size() <= 0){
        Debug::Log("no keypoints detected", Color::Red);
        return -1;
    }

    Debug::Log("keypoints count", Color::Blue);
    Debug::Log((int)keypoints.size());
    
    Debug::Log("descriptor size", Color::Blue);
    Debug::Log((int)descriptors.rows);
    Debug::Log((int)descriptors.cols);


    return 0;
}

void ImageTarget::ExportDatabase()
{
    
    cv::FileStorage file("Assets/Imagetargets/" + name + ".xml", cv::FileStorage::WRITE);
    cv::write(file, "size", size);
    cv::write(file, "keypoints", keypoints);
    cv::write(file, "descriptors", descriptors);
    
    
    stringstream ss;
    ss << "file save complete: " << name << endl;
    Debug::Log(ss);
    //cout << ss.str();
    //Debug::Log("file save complete");
    file.release();
}

void ImageTarget::ImportDatabase(string imgName)
{
    name = imgName;
    stringstream ss;
    ss << "importing db: " << imgName << endl;
    Debug::Log(ss);
    
    cv::FileStorage file("Assets/Imagetargets/" + name + ".xml", cv::FileStorage::READ);

    cv::FileNode sizeNode = file["size"];
    cv::read(sizeNode, size, Size(0,0));
    ss.clear();
    ss << "size [" << size.width << ", " << size.height << "]\n";
    Debug::Log(ss);
    
    cv::FileNode keypointsNode = file["keypoints"];
    cv::read(keypointsNode, keypoints);
    cv::FileNode descriptorsNode = file["descriptors"];
    cv::read(descriptorsNode, descriptors);
    file.release();
    
    points2d.resize(4);
    points3d.resize(4);
    
    // Image dimensions
    const float w = size.width;
    const float h = size.height;
    
    // Normalized dimensions:
    const float maxSize = std::max(w,h);
    const float unitW = w / maxSize;
    const float unitH = h / maxSize;
    
    points2d[0] = cv::Point2f(0,0);
    points2d[1] = cv::Point2f(w,0);
    points2d[2] = cv::Point2f(w,h);
    points2d[3] = cv::Point2f(0,h);
    
    points3d[0] = cv::Point3f(-unitW, -unitH, 0);
    points3d[1] = cv::Point3f( unitW, -unitH, 0);
    points3d[2] = cv::Point3f( unitW,  unitH, 0);
    points3d[3] = cv::Point3f(-unitW,  unitH, 0);
    
    
    //check

    Debug::Log("keypoints size", Color::Blue);
    Debug::Log((int)keypoints.size(), Color::Orange);

    Debug::Log("descriptor size", Color::Blue);
    Debug::Log((int)descriptors.rows);
    Debug::Log((int)descriptors.cols);
    
}


//---------Tracking Info-----------------

void TrackingInfo::drawRawOutline(cv::Mat& image, cv::Scalar color) const
{
    for (size_t i = 0; i < raw_points2d.size(); i++)
    {
        cv::line(image, raw_points2d[i], raw_points2d[ (i+1) % raw_points2d.size() ], color, 2, CV_AA);
    }
}

void TrackingInfo::showAxes(CameraCalibration calib, Transformation tMat, Mat& img)
{
    std::vector<cv::Point3d> pts;
    pts.push_back(Point3d(0,0,0));
    pts.push_back(Point3d(0.5,0,0));
    pts.push_back(Point3d(0,0.5,0));
    pts.push_back(Point3d(0,0,0.5));
    
    std::vector<cv::Point2d> newpts;
    
    cv::projectPoints(pts, tMat.Rvec, tMat.Tvec, calib.getIntrinsic(), calib.getDistorsion(), newpts);
    
    cv::line(img, newpts.at(0), newpts.at(1), Scalar(0,0,255),2);
    cv::line(img, newpts.at(0), newpts.at(2), Scalar(0,255,0),2);
    cv::line(img, newpts.at(0), newpts.at(3), Scalar(255,0,0),2);
    
}

void TrackingInfo::computeRawPose(const ImageTarget& imageTarget, const CameraCalibration& calibration)
{
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux = (Mat_<double>(3,1));
    cv::Mat taux = (Mat_<double>(3,1));
    
    
    cv::solvePnP(imageTarget.points3d, raw_points2d, calibration.getIntrinsic(), calibration.getDistorsion(), raux, taux, false, SOLVEPNP_ITERATIVE);
    raux.convertTo(Rvec,CV_32F);
    taux.convertTo(Tvec ,CV_32F);
    
    raw_pose3d.Rvec = Rvec;
    raw_pose3d.Tvec = Tvec;
    
    cv::Mat_<float> rotMat(3,3);
    cv::Rodrigues(Rvec, rotMat);
    
    // Copy to transformation matrix
    for (int col=0; col<3; col++)
    {
        for (int row=0; row<3; row++)
        {
            raw_pose3d.r().mat[row][col] = rotMat(row,col); // Copy rotation component
        }
        raw_pose3d.t().data[col] = Tvec(col); // Copy translation component
    }
    
    //compute pose from Kalman
    kf_pose3d.Rvec = kf_rvec;
    kf_pose3d.Tvec = kf_tvec;
    cv::Mat_<float> kf_rotMat(3,3);
    cv::Rodrigues(kf_rvec, kf_rotMat);
    
    // Copy to transformation matrix
    for (int col=0; col<3; col++)
    {
        for (int row=0; row<3; row++)
        {
            kf_pose3d.r().mat[row][col] = kf_rotMat(row,col); // Copy rotation component
        }
        kf_pose3d.t().data[col] = kf_tvec.at<float>(col); // Copy translation component
    }
}

void TrackingInfo::predictKalman(){
    double dT = (kf_tick - kf_lastTick) / getTickFrequency();
    predictKalman(dT);
}

void TrackingInfo::predictKalman(const double dT){
    
    kf.transitionMatrix.at<float>(3) = dT;
    kf.transitionMatrix.at<float>(16) = dT;
    kf.transitionMatrix.at<float>(29) = dT;
    kf.transitionMatrix.at<float>(81) = dT;
    kf.transitionMatrix.at<float>(94) = dT;
    kf.transitionMatrix.at<float>(107) = dT;
    //dT's at 3, 16, 29, 81, 94, 107
    
    //cout << "dT: " << dT << endl;
    kf_state = kf.predict();
    
    //project points from prediction
    //kf_tvec = Mat_<float>(3,1);
    kf_tvec.at<float>(0) = kf_state.at<float>(0);
    kf_tvec.at<float>(1) = kf_state.at<float>(1);
    kf_tvec.at<float>(2) = kf_state.at<float>(2);
    
    //kf_rvec = Mat_<float>(3,1);
    kf_rvec.at<float>(0) = kf_state.at<float>(6);
    kf_rvec.at<float>(1) = kf_state.at<float>(7);
    kf_rvec.at<float>(2) = kf_state.at<float>(8);
    
    //fix gimbal locking singularity at z-axis
    float rz = kf_rvec.at<float>(2);
    if(rz > M_PI)
    {
        kf_rvec.at<float>(2) = -2 * M_PI + rz;
        cout << " flip to -rz\n";
        resetKalman();
    }
    else if(rz < -M_PI)
    {
        kf_rvec.at<float>(2) = 2 * M_PI - rz;
        cout << " flip to +rz\n";
        resetKalman();

    }
    //cout << pose3d.Rvec << " : " << kf_rvec;
    
    std::vector<float> tvel;
    std::vector<float> rvel;
    tvel.resize(3);
    rvel.resize(3);

    tvel[0] = kf_state.at<float>(3);
    tvel[1] = kf_state.at<float>(4);
    tvel[3] = kf_state.at<float>(5);
    
    rvel[0] = kf_state.at<float>(9);
    rvel[1] = kf_state.at<float>(10);
    rvel[3] = kf_state.at<float>(11);
    
//    float tvmag = sqrt(tvel[0]*tvel[0] + tvel[1]*tvel[1] + tvel[2]*tvel[2]);
//    float rvmag = sqrt(rvel[0]*rvel[0] + rvel[1]*rvel[1] + rvel[2]*rvel[2]);
//    cout << "tvel = " << tvmag << ",\t";
//    cout << "rvel = " << rvmag << endl;
//    cout << "rz = " << rz << endl;
    
//    steadystate = tvmag * rvmag;
//    cout << "ss: " << steadystate << endl;

    cv::projectPoints(kf_imagetarget.points3d, kf_rvec, kf_tvec, calib.getIntrinsic(), calib.getDistorsion(), kf_projectedpoints);
    
    //findhomography from projected points
    kf_homography = cv::findHomography(kf_imagetarget.points2d, kf_projectedpoints);
}

void TrackingInfo::correctKalman(){
    
//    //fix gimbal locking singularity in z-axis
//    float rz = kf_state.at<float>(2);
//    if(rz > M_PI){
//        resetKalman();
//        //kf_state.at<float>(2) = -2 * M_PI + rz;
//    }
//    else if(rz < -M_PI){
//        resetKalman();
//        //kf_state.at<float>(2) = 2 * M_PI + rz;
//    }
   
    
    kf_meas.at<float>(0) = raw_pose3d.Tvec.at<float>(0);
    kf_meas.at<float>(1) = raw_pose3d.Tvec.at<float>(1);
    kf_meas.at<float>(2) = raw_pose3d.Tvec.at<float>(2);
    
    kf_meas.at<float>(3) = raw_pose3d.Rvec.at<float>(0);
    kf_meas.at<float>(4) = raw_pose3d.Rvec.at<float>(1);
    kf_meas.at<float>(5) = raw_pose3d.Rvec.at<float>(2);
    
    
    kf.correct(kf_meas);
    kf_has_prediction = true;
    
    //cout << "kalman correct\n";
}

void TrackingInfo::updateKalman(){
    kf_lastTick = kf_tick;
    kf_tick = cv::getTickCount();
    
    if(found)
    {
        if(kf_has_prediction)
        {
            predictKalman();
        }
        
        correctKalman();
        
    }
    else
    {
        resetKalman();
    }
}

//pass delta time from Unity
void TrackingInfo::updateKalman(const double dT){
    
    if(found)
    {
        //kf_lostTime = 0;
        if(kf_has_prediction)
        {
            predictKalman(dT);
        }
        
        correctKalman();
        
    }
    else
    {
//        kf_lostTime += dT;
//        if(kf_lostTime >= 0.25)
//        {
//            resetKalman();
//        }
//        else{
//            if(kf_has_prediction)
//                predictKalman();
//        }
        
        resetKalman();
        
    }
}

void TrackingInfo::drawKalmanOutline(cv::Mat &img)
{
    for (size_t i = 0; i < kf_projectedpoints.size(); i++)
    {
        cv::line(img, kf_projectedpoints[i], kf_projectedpoints[ (i+1) % kf_projectedpoints.size() ], Scalar(0,128,0), 2, CV_AA);
        cv::circle(img, kf_projectedpoints[i], 5, Scalar(0,128,0), CV_FILLED);
        //cv::circle(img, kf_imagetarget.points2d[i], 10, Scalar(128, 128,128));
    }
    //cout << "pts: " << kf_projectedpoints << endl;

}

void TrackingInfo::initKalman(const ImageTarget& imageTarget, const CameraCalibration& calibration){
    int stateSize = 12; //tx, ty, tz, v_tx, v_ty, v_tz, rx, ry, rz, v_rx, v_ry, v_rz
    int measSize = 6;//tx', ty', tz', rx', ry', rz'
    int contrSize = 0;
    unsigned int type = CV_32F;
    
    kf = KalmanFilter(stateSize, measSize, contrSize, type);
    kf_state = Mat(stateSize, 1, type);
    kf_meas = Mat(measSize, 1, type);
    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 0 dT 0  0  0 0 0 0  0  0  ]
    // [ 0 1 0 0  dT 0  0 0 0 0  0  0  ]
    // [ 0 0 1 0  0  dT 0 0 0 0  0  0  ]
    // [ 0 0 0 1  0  0  0 0 0 0  0  0  ]
    // [ 0 0 0 0  1  0  0 0 0 0  0  0  ]
    // [ 0 0 0 0  0  1  0 0 0 0  0  0  ]
    // [ 0 0 0 0  0  0  1 0 0 dT 0  0  ]
    // [ 0 0 0 0  0  0  0 1 0 0  dT 0  ]
    // [ 0 0 0 0  0  0  0 0 1 0  0  dT ]
    // [ 0 0 0 0  0  0  0 0 0 1  0  0  ]
    // [ 0 0 0 0  0  0  0 0 0 0  1  0  ]
    // [ 0 0 0 0  0  0  0 0 0 0  0  1  ]
    //dT's at 3, 16, 29, 81, 94, 107
    cv::setIdentity(kf.transitionMatrix);
    
    // Measure Matrix H
    // [ 1 0 0 0 0 0 0 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 0 0 0 0 0 0 ]
    // [ 0 0 1 0 0 0 0 0 0 0 0 0 ]
    // [ 0 0 0 0 0 0 1 0 0 0 0 0 ]
    // [ 0 0 0 0 0 0 0 1 0 0 0 0 ]
    // [ 0 0 0 0 0 0 0 0 1 0 0 0 ]

    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0)  = 1.0f;
    kf.measurementMatrix.at<float>(13) = 1.0f;
    kf.measurementMatrix.at<float>(26)  = 1.0f;
    kf.measurementMatrix.at<float>(42)  = 1.0f;
    kf.measurementMatrix.at<float>(55)  = 1.0f;
    kf.measurementMatrix.at<float>(68)  = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Etx 0   0   0     0     0     0   0   0   0     0     0     ]
    // [ 0   Ety 0   0     0     0     0   0   0   0     0     0     ]
    // [ 0   0   Etz 0     0     0     0   0   0   0     0     0     ]
    // [ 0   0   0   Ev_tx 0     0     0   0   0   0     0     0     ]
    // [ 0   0   0   0     Ev_ty 0     0   0   0   0     0     0     ]
    // [ 0   0   0   0     0     Ev_tz 0   0   0   0     0     0     ]
    // [ 0   0   0   0     0     0     Erx 0   0   0     0     0     ]
    // [ 0   0   0   0     0     0     0   Ery 0   0     0     0     ]
    // [ 0   0   0   0     0     0     0   0   Erz 0     0     0     ]
    // [ 0   0   0   0     0     0     0   0   0   Ev_rx 0     0     ]
    // [ 0   0   0   0     0     0     0   0   0   0     Ev_ry 0     ]
    // [ 0   0   0   0     0     0     0   0   0   0     0     Ev_rz ]
    float nc1 = .0001;
    float nc2 = 1;
    float nc3 = 1;
    kf.processNoiseCov.at<float>(0)   = nc1;
    kf.processNoiseCov.at<float>(13)  = nc1;
    kf.processNoiseCov.at<float>(26)  = nc1;
    kf.processNoiseCov.at<float>(39)  = nc2;
    kf.processNoiseCov.at<float>(52)  = nc2;
    kf.processNoiseCov.at<float>(65)  = nc2;
    kf.processNoiseCov.at<float>(78)  = nc1;
    kf.processNoiseCov.at<float>(91)  = nc1;
    kf.processNoiseCov.at<float>(104) = nc1;
    kf.processNoiseCov.at<float>(117) = nc3;
    kf.processNoiseCov.at<float>(130) = nc3;
    kf.processNoiseCov.at<float>(143) = nc3;
    
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    
    kf_tick = 0;
    kf_has_prediction = false;
    
    kf_imagetarget = imageTarget;
    
    calib = calibration;
}

void TrackingInfo::resetKalman(){
    // >>>> Initialization
    kf.errorCovPre.at<float>(0) = 1;
    kf.errorCovPre.at<float>(13) = 1;
    kf.errorCovPre.at<float>(26) = 1;
    kf.errorCovPre.at<float>(39) = 1;
    kf.errorCovPre.at<float>(52) = 1;
    kf.errorCovPre.at<float>(65) = 1;
    kf.errorCovPre.at<float>(78) = 1;
    kf.errorCovPre.at<float>(91) = 1;
    kf.errorCovPre.at<float>(104) = 1;
    kf.errorCovPre.at<float>(117) = 1;
    kf.errorCovPre.at<float>(130) = 1;
    kf.errorCovPre.at<float>(143) = 1;
    
    kf_state.at<float>(0)  = kf_meas.at<float>(0);
    kf_state.at<float>(1)  = kf_meas.at<float>(1);
    kf_state.at<float>(2)  = kf_meas.at<float>(2);
    kf_state.at<float>(3)  = 0;
    kf_state.at<float>(4)  = 0;
    kf_state.at<float>(5)  = 0;
    kf_state.at<float>(6)  = kf_meas.at<float>(3);
    kf_state.at<float>(7)  = kf_meas.at<float>(4);
    kf_state.at<float>(8)  = kf_meas.at<float>(5);
    kf_state.at<float>(9)  = 0;
    kf_state.at<float>(10) = 0;
    kf_state.at<float>(11) = 0;
    // <<<< Initialization
    
    kf.statePost = kf_state;
    
    kf_has_prediction = false;
    kf_homography = raw_homography;
}

void TrackingInfo::finalizeHomography(){
    
}

void TrackingInfo::drawFinalOutline(cv::Mat &img)
{
    
}

