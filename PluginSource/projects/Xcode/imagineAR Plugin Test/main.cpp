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
#include "GL/glew.h"
#include "Tracker.hpp"
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"
#include "ARUtils.hpp"
#include "GLFW/glfw3.h"

void LoadImageToDatabase()
{
    cv::Mat img;
    img = cv::imread("panda.jpg");

    ARUtils::Resize(img, img);

    ImageTarget imagetarget;
    imagetarget.BuildFromImage(img, "panda");
    imagetarget.ExportDatabase();
}


int main(int argc, const char * argv[]) {

    LoadImageToDatabase();

    //return 0;

    ImageTarget imageTarget;
    imageTarget.ImportDatabase("panda");
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
    trainImg = cv::imread("panda.jpg");
    ARUtils::Resize(trainImg, trainImg);

    Mat webcamImage;
    cap >> webcamImage;
    cv::Size min = ARUtils::GetScaledSize(cv::Size(webcamImage.cols, webcamImage.rows));
    CameraCalibration calib(min.width, min.width, min.width / 2, min.height / 2);

    tracker.m_trackingInfo.initKalman(imageTarget, calib);
    
    for(;;)
    {
        cv::Mat frame, small, gray, debugMatches, masked, mask;
        cap >> frame;

        ARUtils::Resize(frame, small);

        if(!tracker.homographyFoundInLastFrame)
            ARUtils::GetGraySharp(small, gray);
        else
            ARUtils::GetGraySharp(small, gray);

        bool found = tracker.findPattern(gray);

        

        cv::drawMatches( gray, tracker.m_queryKeypoints, trainImg, imageTarget.keypoints,
                        tracker.m_matches, debugMatches, Scalar(0,255,0), Scalar(0,0,255),
                        vector<char>(), DrawMatchesFlags::DEFAULT );


        if(found){
            tracker.m_trackingInfo.computeRawPose(imageTarget, calib);

            //show masked region
            
            //show kalman filtered homography
            if(!tracker.m_trackingInfo.kf_homography.empty() && found)
            {
                tracker.m_trackingInfo.drawKalmanOutline(debugMatches);
                cv:Mat kf_warped;
                cv::warpPerspective(gray, kf_warped, tracker.m_trackingInfo.kf_homography, tracker.m_trackingInfo.kf_imagetarget.size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
                
                cv::imshow("kf_warped", kf_warped);
            }

            //show outline
            tracker.m_trackingInfo.drawRawOutline(debugMatches, Scalar(0,255,255));
            //show axes
            tracker.m_trackingInfo.showAxes(calib, tracker.m_trackingInfo.raw_pose3d, debugMatches);

            //tracker.m_trackingInfo.computeRawPose(imageTarget, calib);
            
            
        }

        tracker.m_trackingInfo.updateKalman();

        cv::imshow("vid", debugMatches);

        //if(!tracker.m_warpedImg2.empty())
        //   cv::imshow("warped", tracker.m_warpedImg2);

        if(waitKey(30) >= 0) break;
    }

    return 0;
}
