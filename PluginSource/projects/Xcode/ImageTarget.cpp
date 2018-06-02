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

int ImageTarget::BuildFromImage(const Mat& scaledImage, string imgName){
    bool result = true;
    
    name = imgName;
    image = scaledImage.clone();
    size = Size(image.cols, image.rows);
    GetGray(image, grayImg);
    
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
    Ptr<FeatureDetector> detector  = ORB::create(1000);
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
    
    cv::FileStorage file(name + ".xml", cv::FileStorage::WRITE);
    cv::write(file, "size", size);
    cv::write(file, "keypoints", keypoints);
    cv::write(file, "descriptors", descriptors);
    
    Debug::Log("file save complete");
    file.release();
}

void ImageTarget::ImportDatabase(string imgName)
{
    
    name = imgName;
    
    cv::FileStorage file(name + ".xml", cv::FileStorage::READ);

    cv::FileNode sizeNode = file["size"];
    cv::read(sizeNode, size, Size(0,0));
    
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

void ImageTarget::GetGray(const cv::Mat& image, cv::Mat& gray)
{
    if (image.channels()  == 3)
        cv::cvtColor(image, gray, CV_BGR2GRAY);
    else if (image.channels() == 4)
        cv::cvtColor(image, gray, CV_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
}

//---------Tracking Info-----------------

void TrackingInfo::draw2dContour(cv::Mat& image, cv::Scalar color) const
{
    for (size_t i = 0; i < points2d.size(); i++)
    {
        cv::line(image, points2d[i], points2d[ (i+1) % points2d.size() ], color, 2, CV_AA);
    }
}

void TrackingInfo::computePose(const ImageTarget& imageTarget, const CameraCalibration& calibration)
{
    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux,taux;
    
    cv::solvePnP(imageTarget.points3d, points2d, calibration.getIntrinsic(), calibration.getDistorsion(),raux,taux);
    raux.convertTo(Rvec,CV_32F);
    taux.convertTo(Tvec ,CV_32F);
    
    pose3d.Rvec = Rvec.clone();
    pose3d.Tvec = Tvec.clone();
    
    cv::Mat_<float> rotMat(3,3);
    cv::Rodrigues(Rvec, rotMat);
    
    // Copy to transformation matrix
    for (int col=0; col<3; col++)
    {
        for (int row=0; row<3; row++)
        {
            pose3d.r().mat[row][col] = rotMat(row,col); // Copy rotation component
        }
        pose3d.t().data[col] = Tvec(col); // Copy translation component
    }
    
    
    // Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
    //pose3d = pose3d.getInverted();
}
