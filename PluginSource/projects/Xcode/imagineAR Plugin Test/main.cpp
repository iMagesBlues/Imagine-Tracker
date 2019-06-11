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


///KALMAN
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/highgui.hpp"
//#include <stdio.h>
//using namespace cv;
//static inline Point calcPoint(Point2f center, double R, double angle)
//{
//    return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
//}
//static void help()
//{
//    printf( "\nExample of c calls to OpenCV's Kalman filter.\n"
//           "   Tracking of rotating point.\n"
//           "   Rotation speed is constant.\n"
//           "   Both state and measurements vectors are 1D (a point angle),\n"
//           "   Measurement is the real point angle + gaussian noise.\n"
//           "   The real and the estimated points are connected with yellow line segment,\n"
//           "   the real and the measured points are connected with red line segment.\n"
//           "   (if Kalman filter works correctly,\n"
//           "    the yellow segment should be shorter than the red one).\n"
//           "\n"
//           "   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
//           "   Pressing ESC will stop the program.\n"
//           );
//}
//int main(int, char**)
//{
//    help();
//    Mat img(500, 500, CV_8UC3);
//    KalmanFilter KF(2, 1, 0);
//    Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
//    Mat processNoise(2, 1, CV_32F);
//    Mat measurement = Mat::zeros(1, 1, CV_32F);
//    char code = (char)-1;
//    for(;;)
//    {
//        randn( state, Scalar::all(0), Scalar::all(0.1) );
//        KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);
//        setIdentity(KF.measurementMatrix);
//        setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
//        setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
//        setIdentity(KF.errorCovPost, Scalar::all(1));
//        randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));
//        for(;;)
//        {
//            Point2f center(img.cols*0.5f, img.rows*0.5f);
//            float R = img.cols/3.f;
//            double stateAngle = state.at<float>(0);
//            Point statePt = calcPoint(center, R, stateAngle);
//            Mat prediction = KF.predict();
//            double predictAngle = prediction.at<float>(0);
//            Point predictPt = calcPoint(center, R, predictAngle);
//            randn( measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));
//            // generate measurement
//            measurement += KF.measurementMatrix*state;
//            double measAngle = measurement.at<float>(0);
//            Point measPt = calcPoint(center, R, measAngle);
//            // plot points
//#define drawCross( center, color, d )                                        \
//line( img, Point( center.x - d, center.y - d ),                          \
//Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
//line( img, Point( center.x + d, center.y - d ),                          \
//Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )
//            img = Scalar::all(0);
//            drawCross( statePt, Scalar(255,255,255), 3 );
//            drawCross( measPt, Scalar(0,0,255), 3 );
//            drawCross( predictPt, Scalar(0,255,0), 3 );
//            line( img, statePt, measPt, Scalar(0,0,255), 3, LINE_AA, 0 );
//            line( img, statePt, predictPt, Scalar(0,255,255), 3, LINE_AA, 0 );
//            if(theRNG().uniform(0,4) != 0)
//                KF.correct(measurement);
//            randn( processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
//            state = KF.transitionMatrix*state + processNoise;
//            imshow( "Kalman", img );
//            code = (char)waitKey(100);
//            if( code > 0 )
//                break;
//        }
//        if( code == 27 || code == 'q' || code == 'Q' )
//            break;
//    }
//    return 0;
//}
