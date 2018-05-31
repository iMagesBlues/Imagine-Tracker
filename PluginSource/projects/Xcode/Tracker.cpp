

////////////////////////////////////////////////////////////////////
// File includes:
#include "Tracker.hpp"
////////////////////////////////////////////////////////////////////
// Standard includes:
#include <cmath>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <cassert>
#include "DebugCPP.hpp"

Tracker::Tracker(cv::Ptr<cv::FeatureDetector> detector,
    cv::Ptr<cv::DescriptorExtractor> extractor, 
    cv::Ptr<cv::DescriptorMatcher> matcher, 
    bool ratioTest)
    : m_detector(detector)
    , m_extractor(extractor)
    , m_matcher(matcher)
    , enableRatioTest(ratioTest)
    , enableHomographyRefinement(true)
    , homographyReprojectionThreshold(15)
{
}


void Tracker::train(const ImageTarget& imageTarget)
{
    // Store the pattern object
    m_imageTarget = imageTarget;

    // API of cv::DescriptorMatcher is somewhat tricky
    // First we clear old train data:
    m_matcher->clear();
    
    // Then we add vector of descriptors (each descriptors matrix describe one image). 
    // This allows us to perform search across multiple images:
    std::vector<cv::Mat> descriptors(1);
    descriptors[0] = imageTarget.descriptors;
    m_matcher->add(descriptors);
    
    // After adding train data perform actual train:
    m_matcher->train();
}



bool Tracker::findPattern(const cv::Mat& image)
{
    m_grayImg = image;
    
    // Extract feature points from input gray image
    extractFeatures(m_grayImg, m_queryKeypoints, m_queryDescriptors);
    
    // Get matches with current pattern
    getMatches(m_queryDescriptors, m_matches);
    
    
    // Find homography transformation and detect good matches
    bool homographyFound = refineMatchesWithHomography(
        m_queryKeypoints, 
        m_imageTarget.keypoints,
        homographyReprojectionThreshold, 
        m_matches, 
        m_roughHomography);
    

    if (homographyFound)
    {

        // If homography refinement enabled improve found transformation
        if (enableHomographyRefinement)
        {
            // Warp image using found homography
            cv::warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_imageTarget.size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);

            // Get refined matches:
            std::vector<cv::KeyPoint> warpedKeypoints;
            std::vector<cv::DMatch> refinedMatches;

            // Detect features on warped image
            extractFeatures(m_warpedImg, warpedKeypoints, m_queryDescriptors);

            // Match with pattern
            getMatches(m_queryDescriptors, refinedMatches);

            // Estimate new refinement homography
            homographyFound = refineMatchesWithHomography(
                warpedKeypoints, 
                m_imageTarget.keypoints,
                homographyReprojectionThreshold, 
                refinedMatches, 
                m_refinedHomography);
            
            
            if(!m_refinedHomography.empty())
            {
                
                //use second warp
                
                Mat m_warpedImg2, m_refinedHomography2;
                // Warp image using found homography
                cv::warpPerspective(m_warpedImg, m_warpedImg2, m_refinedHomography, m_imageTarget.size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
                
                // Get refined matches:
                std::vector<cv::KeyPoint> warpedKeypoints2;
                std::vector<cv::DMatch> refinedMatches2;
                
                // Detect features on warped image
                extractFeatures(m_warpedImg2, warpedKeypoints2, m_queryDescriptors);
                
                // Match with pattern
                getMatches(m_queryDescriptors, refinedMatches2);
                
                // Estimate new refinement homography
                homographyFound = refineMatchesWithHomography(
                                                              warpedKeypoints2,
                                                              m_imageTarget.keypoints,
                                                              homographyReprojectionThreshold,
                                                              refinedMatches2,
                                                              m_refinedHomography2);
                if(!m_refinedHomography2.empty())
                {
                    // Get a result homography as result of matrix product of refined and rough homographies:
                    m_trackingInfo.homography = m_roughHomography * m_refinedHomography * m_refinedHomography2;
                    
                    stringstream ss;
                    ss << "homography second refinement successful " << std::to_string(refinedMatches2.size()) << endl;
                    Debug::Log(ss);
                    cout << ss.str();
                    
                    
                    // Transform contour with precise homography
                    cv::perspectiveTransform(m_imageTarget.points2d, m_trackingInfo.points2d, m_trackingInfo.homography);
                }
                else
                {
                
                // Get a result homography as result of matrix product of refined and rough homographies:
                m_trackingInfo.homography = m_roughHomography * m_refinedHomography;
                m_warpedImg = m_warpedImg2;
                    
                stringstream ss;
                ss << "homography refinement successful " << std::to_string(refinedMatches.size()) << endl;
                Debug::Log(ss);
                cout << ss.str();
             

                // Transform contour with precise homography
                cv::perspectiveTransform(m_imageTarget.points2d, m_trackingInfo.points2d, m_trackingInfo.homography);
                }
            }
            else
            {
                stringstream ss;
                ss << "homography refinement failed " << std::to_string(m_matches.size()) << endl;
                Debug::Log(ss);
                cout << ss.str();
                
                m_trackingInfo.homography = m_roughHomography;
                // Transform contour with precise homography
                cv::perspectiveTransform(m_imageTarget.points2d, m_trackingInfo.points2d, m_trackingInfo.homography);
            }
        }
        else
        {
            m_trackingInfo.homography = m_roughHomography;

            // Transform contour with rough homography
            cv::perspectiveTransform(m_imageTarget.points2d, m_trackingInfo.points2d, m_roughHomography);
            
            stringstream ss;
            ss << "homography found " << std::to_string(m_matches.size()) << endl;
            Debug::Log(ss);
            cout << ss.str();
        }        
    }
    else
    {
        stringstream ss;
        ss << "homography not found " << std::to_string(m_matches.size()) << endl;
        Debug::Log(ss);
        cout << ss.str();
    }

    return homographyFound;
}


bool Tracker::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const
{
    assert(!image.empty());
    assert(image.channels() == 1);

    //TODO:use detect and compute with mask to eliminate keypoints outside
    m_detector->detect(image, keypoints);
    if (keypoints.empty())
        return false;
    
    m_extractor->compute(image, keypoints, descriptors);
    if (keypoints.empty())
        return false;

    return true;
}



void Tracker::getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches)
{
    matches.clear();
    m_knnMatches.clear();

    if (enableRatioTest)
    {
        // To avoid NaN's when best match has zero distance we will use inversed ratio. 
        const float minRatio = 1.f / 1.5f;
        
        // KNN match will return 2 nearest matches for each query descriptor
        m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);

        for (size_t i=0; i<m_knnMatches.size(); i++)
        {
            const cv::DMatch& bestMatch   = m_knnMatches[i][0];
            const cv::DMatch& betterMatch = m_knnMatches[i][1];

            float distanceRatio = bestMatch.distance / betterMatch.distance;
            
            // Pass only matches where distance ratio between 
            // nearest matches is greater than 1.5 (distinct criteria)
            if (distanceRatio < minRatio)
            {
                matches.push_back(bestMatch);
            }
        }
        
    }
    else
    {
        // Perform regular match
        m_matcher->match(queryDescriptors, matches);
    }
}


bool Tracker::refineMatchesWithHomography
    (
    const std::vector<cv::KeyPoint>& queryKeypoints,
    const std::vector<cv::KeyPoint>& trainKeypoints, 
    float reprojectionThreshold,
    std::vector<cv::DMatch>& matches,
    cv::Mat& homography
    )
{
    const int minNumberMatchesAllowed = 4;

    if (matches.size() < minNumberMatchesAllowed)
        return false;
    
    if(queryKeypoints.size() <= 0)
        return false;

    // Prepare data for cv::findHomography
    std::vector<cv::Point2f> srcPoints(matches.size());
    std::vector<cv::Point2f> dstPoints(matches.size());

    for (size_t i = 0; i < matches.size(); i++)
    {
        srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
        dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
    }

    // Find homography matrix and get inliers mask
    std::vector<unsigned char> inliersMask(srcPoints.size());
    homography = cv::findHomography(srcPoints, 
                                    dstPoints, 
                                    CV_FM_RANSAC,
                                    reprojectionThreshold,
                                    inliersMask);

    std::vector<cv::DMatch> inliers;
    for (size_t i=0; i<inliersMask.size(); i++)
    {
        if (inliersMask[i])
            inliers.push_back(matches[i]);
    }

    matches.swap(inliers);
    return matches.size() > minNumberMatchesAllowed;
}
