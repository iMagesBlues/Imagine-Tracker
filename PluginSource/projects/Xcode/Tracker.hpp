//
//  Tracker.hpp
//  libimaginear
//
//  Created by Ronnie Miguel Besas on 5/13/18.
//  Copyright © 2018 Ronnie Miguel Besas. All rights reserved.
//
#ifndef Tracker_hpp
#define Tracker_hpp

#include "ImageTarget.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>


class Tracker
{
public:
    /**
     * Initialize a pattern detector with specified feature detector, descriptor extraction and matching algorithm
     */
    Tracker
        (
         cv::Ptr<cv::FeatureDetector>     detector  = cv::ORB::create(1000),
         cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create(1000),
         cv::Ptr<cv::DescriptorMatcher>   matcher   = cv::DescriptorMatcher::create(cv::BFMatcher::BRUTEFORCE_HAMMING),
         //cv::Ptr<cv::DescriptorMatcher>   matcher   = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED),
         bool enableRatioTest                       = true
        );

    /**
    * 
    */
    void train(const ImageTarget& imageTarget);
    
    bool findPattern(const cv::Mat& image);

    bool enableRatioTest;
    bool enableHomographyRefinement;
    float homographyReprojectionThreshold;
    
    //exposed for debugging
    TrackingInfo m_trackingInfo;
    std::vector<cv::KeyPoint> m_queryKeypoints;
    std::vector<cv::DMatch>   m_matches;
    std::vector<cv::DMatch>   last_matches;
    cv::Mat                   m_warpedImg;


protected:

    bool extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;

    void getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches);
    
    static bool refineMatchesWithHomography(
        const std::vector<cv::KeyPoint>& queryKeypoints, 
        const std::vector<cv::KeyPoint>& trainKeypoints, 
        float reprojectionThreshold,
        std::vector<cv::DMatch>& matches, 
        cv::Mat& homography);

private:
    cv::Mat                   m_queryDescriptors;
    std::vector< std::vector<cv::DMatch> > m_knnMatches;

    cv::Mat                   m_grayImg;
    cv::Mat                   m_roughHomography;
    cv::Mat                   m_refinedHomography;

    ImageTarget                      m_imageTarget;
    cv::Ptr<cv::FeatureDetector>     m_detector;
    cv::Ptr<cv::DescriptorExtractor> m_extractor;
    cv::Ptr<cv::DescriptorMatcher>   m_matcher;
};

#endif /* Tracker_hpp */
