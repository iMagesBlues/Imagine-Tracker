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

void LoadImageToDatabase()
{
    cv::Mat img;
    img = cv::imread("panda.jpg");
    
    int minW = 300;
    int minH = (float)(img.rows * minW) / img.cols;
    
    cv::resize(img, img, Size(minW, minH));
    
    ImageTarget imagetarget;
    imagetarget.BuildFromImage(img, "pandatest");
    imagetarget.ExportDatabase();
}

int main(int argc, const char * argv[]) {
    

    //LoadImageToDatabase();
        
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
    trainImg = cv::imread("panda.jpg");
    int minW = 300;
    int minH = (float)(trainImg.rows * minW) / trainImg.cols;
    cv::resize(trainImg, trainImg, Size(minW, minH));

    for(;;)
    {
        cv::Mat frame, small, gray, debugMatches;
        cap >> frame;
        cv::resize(frame, small, Size(300,169));
        cv::cvtColor(small, gray, CV_BGR2GRAY);

        tracker.findPattern(gray);

        tracker.m_trackingInfo.draw2dContour(gray, Scalar(255,255,0));
        
        //cv::drawMatches(gray, tracker.m_queryKeypoints, trainImg, imageTarget.keypoints, tracker.m_matches, debugMatches, DrawMatchesFlags::DRAW_OVER_OUTIMG);
        
        cv::drawMatches( gray, tracker.m_queryKeypoints, trainImg, imageTarget.keypoints,
                    tracker.m_matches, debugMatches, Scalar(0,255,0), Scalar(255,0,0),
                    vector<char>(), DrawMatchesFlags::DEFAULT );

        cv::imshow("vid", debugMatches);
        if(!tracker.m_warpedImg.empty())
            cv::imshow("warped", tracker.m_warpedImg);

        if(waitKey(30) >= 0) break;
    }
    
    return 0;
}
