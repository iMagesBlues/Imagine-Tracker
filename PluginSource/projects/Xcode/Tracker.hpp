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
         cv::Ptr<cv::DescriptorMatcher>   matcher   = cv::DescriptorMatcher::create(cv::BFMatcher::BRUTEFORCE_HAMMINGLUT),
         //cv::Ptr<cv::DescriptorMatcher>   matcher   = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED),
         bool enableRatioTest                       = true,
         
         
         //KLT
         TermCriteria m_termcrit = TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03),
         cv::Size m_subPixWinSize = cv::Size(10,10),
         cv::Size m_winSize       = cv::Size(31,31)

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
    cv::Mat                   m_warpedImg;
    
    
    //KLT Tracker vars
    std::vector<cv::DMatch> m_kltMatches;
    std::vector<cv::KeyPoint> m_kltPoints;
    cv::Mat m_lastImage;
    
    cv::TermCriteria m_termcrit;
    cv::Size m_subPixWinSize;
    cv::Size m_winSize;
    cv::Mat m_mask;

protected:

    bool extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;

    void getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches);
    
    static bool refineMatchesWithHomography(
        const std::vector<cv::KeyPoint>& queryKeypoints, 
        const std::vector<cv::KeyPoint>& trainKeypoints, 
        float reprojectionThreshold,
        std::vector<cv::DMatch>& matches, 
        cv::Mat& homography);
    
    void computeKLT(cv::Mat image);
    void injectKLTMatches(std::vector<KeyPoint> &queryKeypoints, std::vector<DMatch> &matches);

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
