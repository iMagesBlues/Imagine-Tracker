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

int ImageTarget::BuildFromImage(const Mat& scaledImage, string imgName){
    bool result = true;
    
    name = name;
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

void ImageTarget::ExportDatabase(char* data, int* size)
{
    int keypointSize = sizeof(keypoints);
    int descriptorSize = sizeof(descriptors);
    
    *size = keypointSize + descriptorSize + sizeof(int) + sizeof(int);
    
    memcpy(data,                    &keypointSize,   sizeof(int));
    memcpy(data + 4,                &keypoints,      sizeof(keypoints));
    memcpy(data + keypointSize + 4, &descriptorSize, sizeof(int));
    memcpy(data + keypointSize + 8, &descriptors,    sizeof(descriptors));
    
    Debug::Log("2 keypoints size", Color::Blue);
    Debug::Log((int)*data);
    
    
    Debug::Log("2 descriptors size", Color::Blue);
    Debug::Log((int)*(data + keypointSize + 4));
    
    //TODO: Add unity readable section to access keypoints and quality level
    
}

void ImageTarget::ImportDatabase(char *data, int size)
{
    //get keypoints from data dump
    int keypointSize = (int)data[0];
    vector<KeyPoint> *keypointsPtr = (vector<KeyPoint>*)malloc(keypointSize);
    memcpy(keypointsPtr, data + 4, keypointSize);
    keypoints = *keypointsPtr;

    //get descriptor
    int descriptorSize = (int)data[keypointSize + 4];
    Mat* descriptorsPtr = (Mat*)malloc(descriptorSize);
    memcpy(descriptorsPtr, data + keypointSize + 8, descriptorSize);
    descriptors = *descriptorsPtr;
    
    //check
    Debug::Log((int)keypointSize, Color::Orange);
    Debug::Log((int)sizeof(keypoints), Color::Orange);
    Debug::Log("keypoints size", Color::Blue);
    Debug::Log((int)keypoints.size(), Color::Orange);
    
    
    Debug::Log((int)descriptorSize);
    Debug::Log((int)sizeof(descriptors));
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

void TrackingInfo::draw2dContour(cv::Mat& image, cv::Scalar color) const
{
    for (size_t i = 0; i < points2d.size(); i++)
    {
        cv::line(image, points2d[i], points2d[ (i+1) % points2d.size() ], color, 2, CV_AA);
    }
}
