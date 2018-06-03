//
//  main.cpp
//  imagineAR Plugin Test
//
//  Created by Ronnie Miguel Besas on 5/31/18.
//

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "ImageTarget.hpp"
#include "glew.h"
#include "Tracker.hpp"
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"
#include "ARUtils.hpp"

void LoadImageToDatabase()
{
    cv::Mat img;
    img = cv::imread("panda.jpg");
    
    ARUtils::Resize(img, img);
    
    ImageTarget imagetarget;
    imagetarget.BuildFromImage(img, "pandatest");
    imagetarget.ExportDatabase();
}



int main(int argc, const char * argv[]) {
    

    //LoadImageToDatabase();
    
    //return 0;
        
    ImageTarget imageTarget;
    imageTarget.ImportDatabase("pandatest");
    cout << "imagetarget imported with " << std::to_string(imageTarget.keypoints.size()) + " keypoints" << endl;
    
    Tracker tracker;
    tracker.train(imageTarget);
    
    cout << "tracker trained" << endl;
    
    cv::VideoCapture cap(0);
    if(!cap.isOpened())
        return -1;
    cout << "video initialized" <<endl;
    cv::namedWindow("vid", 1);
    cv::namedWindow("warped");
    
    cv::Mat trainImg;
    trainImg = cv::imread("trainImg.jpg");
    ARUtils::Resize(trainImg, trainImg);
    
    Mat webcamImage;
    cap >> webcamImage;
    cv::Size min = ARUtils::GetScaledSize(cv::Size(webcamImage.cols, webcamImage.rows));
    CameraCalibration calib(min.width, min.width, min.width / 2, min.height / 2);
    
    for(;;)
    {
        cv::Mat frame, small, gray, debugMatches;
        cap >> frame;

        ARUtils::Resize(frame, small);
        ARUtils::GetGray(small, gray);

        bool found = tracker.findPattern(gray);
        
        cv::drawMatches( gray, tracker.m_queryKeypoints, trainImg, imageTarget.keypoints,
                    tracker.m_matches, debugMatches, Scalar(0,255,0), Scalar(0,0,255),
                    vector<char>(), DrawMatchesFlags::DEFAULT );

        if(found){
            
            tracker.m_trackingInfo.computePose(imageTarget, calib);
            tracker.m_trackingInfo.draw2dContour(debugMatches, Scalar(0,255,255));
            tracker.m_trackingInfo.showAxes(calib, tracker.m_trackingInfo.pose3d, debugMatches);
        }
        
        cv::imshow("vid", debugMatches);
        if(!tracker.m_warpedImg.empty())
            cv::imshow("warped", tracker.m_warpedImg);

        if(waitKey(30) >= 0) break;
    }
    
    return 0;
}
