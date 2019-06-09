

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
#include "ARUtils.hpp"

Tracker::Tracker(cv::Ptr<cv::FeatureDetector> detector,
    cv::Ptr<cv::DescriptorExtractor> extractor, 
    cv::Ptr<cv::DescriptorMatcher> matcher, 
    bool ratioTest,
    TermCriteria m_termcrit,
    cv::Size m_subPixWinSize,
    cv::Size m_winSize
    )
    : m_detector(detector)
    , m_extractor(extractor)
    , m_matcher(matcher)
    , enableRatioTest(ratioTest)
    , enableHomographyRefinement(true)
    , homographyReprojectionThreshold(15)
{
    minNumberMatchesAllowed = 5;
    lostCtr = 1;
    enableSecondHomographyRefinement = true;
    homographyFoundInLastFrame = false;
    homographyRefined = false;
    homographyRefinedTwice = false;
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
    Mat best_homography;
    
    m_grayImg = image;
    
    
    if(homographyFoundInLastFrame) //track
    {
        //warp keypoints
        std::vector<cv::KeyPoint> keypts(m_imageTarget.keypoints);
        std::vector<DMatch> matches;

        
        //get pts
        std::vector<Point2f> pts(m_imageTarget.keypoints.size());
        for(int i = 0; i < pts.size(); i++)
        {
            pts[i] = keypts[i].pt;
        }
        //warp
        //cout << "kf_homo: " << m_trackingInfo.kf_homography.size() << endl;
        if(!m_trackingInfo.kf_homography.empty() && m_trackingInfo.kf_has_prediction){
            Mat feedbackHomo = m_trackingInfo.kf_homography;
            perspectiveTransform(pts, pts, feedbackHomo);
        }
        else
            perspectiveTransform(pts, pts, m_trackingInfo.raw_homography);
        //set pts
        for(int i = 0; i < pts.size(); i++)
        {
            keypts[i].pt = pts[i];
            matches.push_back(DMatch(i, i, 0));//fake but perfect matches
        }
        m_queryKeypoints.swap(keypts);
        m_matches.swap((matches));
        
        
    }
    else //detect
    {
        // Extract feature points from input gray image
        extractFeaturesMasked(m_grayImg, m_queryKeypoints, m_queryDescriptors);
        // Get matches with current pattern
        getMatches(m_queryDescriptors, m_matches);
        
        // Inject Recovered Matches from KLT prediction
        //injectKLTMatches(m_queryKeypoints, m_matches);
    }
    
    
    
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
        if (enableHomographyRefinement)// && !homographyRefined)
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
            /*Size size1 = Size(m_grayImg.cols, m_grayImg.rows);
            getGMSMatches(size1,
                          m_imageTarget.size,
                          m_queryKeypoints,
                          m_imageTarget.keypoints,
                          m_queryDescriptors,
                          refinedMatches);*/

            // Estimate new refinement homography
            homographyFound = refineMatchesWithHomography(
                warpedKeypoints, 
                m_imageTarget.keypoints,
                homographyReprojectionThreshold, 
                refinedMatches, 
                m_refinedHomography);
            
            
            if(!m_refinedHomography.empty() && enableSecondHomographyRefinement)// && !homographyRefinedTwice)
            {
                homographyRefined = true;
                
                //use second warp
                
                Mat m_refinedHomography2;
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
                    homographyRefinedTwice = true;
                    //don't use third warp because it's expensive
                    // Get a result homography as result of matrix product of refined and rough homographies:
                    best_homography = m_roughHomography * m_refinedHomography * m_refinedHomography2;
                    
                }
                else
                {
                    homographyRefinedTwice = false;
                    
                    // Get a result homography as result of matrix product of refined and rough homographies:
                    best_homography = m_roughHomography * m_refinedHomography;
                    m_warpedImg = m_warpedImg2;
                }
            }
            else
            {
                homographyRefined = false;
                
                best_homography = m_roughHomography;
            }
        }
        else
        {
            best_homography = m_roughHomography;

            // Transform contour with rough homography        
        }
        
    }
    else
    {

        //stringstream ss;
        //ss << "homography not found " << std::to_string(m_matches.size()) << endl;
        //Debug::Log(ss);
        //cout << ss.str();
    }

    
    
    //create mask roi based on current homography
    if(homographyFound)
    {
        //m_trackingInfo.maskROI = createMaskROI();
        
        
        m_trackingInfo.raw_homography = best_homography - 0.1 * m_trackingInfo.raw_homography;
        
        cv::perspectiveTransform(m_imageTarget.points2d, m_trackingInfo.raw_points2d, m_trackingInfo.raw_homography);
        
        //use KLT to recover matches lost due to high motion
        //computeKLT(image);
    }
    else
    {
        resetKLT();
        
        //m_trackingInfo.maskROI = resetMaskROI();
        
    }
    
    //cout << "matches count: " << m_matches.size() << ". kp size =  " << m_queryKeypoints.size() << endl;
    
    homographyFoundInLastFrame = homographyFound;
    m_trackingInfo.found = homographyFound;
    return homographyFound;
}

void Tracker::injectKLTMatches(std::vector<KeyPoint> &queryKeypoints, std::vector<DMatch> &matches){
    size_t startIndex = queryKeypoints.size();
    std::vector<int> lookup;

    for(int i = 0; i < matches.size(); i++)
    {
        lookup.push_back(matches[i].trainIdx);
    }
    int ctr = 0;
    for (size_t i=0; i < m_kltPoints.size(); i++)
    {
        if(std::find(lookup.begin(), lookup.end(), m_kltMatches[i].trainIdx) != lookup.end() == false)
        {
            queryKeypoints.push_back(m_kltPoints[i]);
            matches.push_back(DMatch((int)(startIndex + ctr), m_kltMatches[i].trainIdx, m_kltMatches[i].distance));
            lookup.push_back(m_kltMatches[i].trainIdx);
            ctr++;
        }
    }
    cout << "injected " << ctr << " matches. ";
}

void Tracker::computeKLT(cv::Mat image){
    
    //predict matches using KLT
    m_kltMatches.clear();
    m_kltPoints.clear();
    
    if(!m_lastImage.empty() && !m_matches.empty()){
        std::vector<cv::Point2f> remainingPoints(m_matches.size());
        std::vector<float> remainingSizes(m_matches.size());
        
        for (size_t i = 0; i < m_matches.size(); i++)
        {
            remainingPoints[i] = m_queryKeypoints[m_matches[i].queryIdx].pt;
            remainingSizes[i] = m_queryKeypoints[m_matches[i].queryIdx].size;
        }
        std::vector<cv::Point2f> predictPoints(m_matches.size());
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(m_lastImage, image, remainingPoints, predictPoints, status, err, Size(31,31),
                             3, TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03), 0, 0.001);
        
        //filter good points
        for (size_t i=0; i < status.size(); i++)
        {
            if (status[i])
            {
                m_kltPoints.push_back(KeyPoint(predictPoints[i], remainingSizes[i]));
                m_kltMatches.push_back(DMatch( m_matches[i].queryIdx, m_matches[i].trainIdx, m_matches[i].distance));
                
            }
        }
        
        //cout << "predicted " << m_kltMatches.size() << " matches from klt\n";

    }
    else{
        //cout << "first frame - no klt\n";
    }
    
    
    m_lastImage = image;
}

void Tracker::resetKLT(){
    m_kltMatches.clear();
    m_kltPoints.clear();
    m_matches.clear();
}

bool Tracker::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const
{
    assert(!image.empty());
    assert(image.channels() == 1);

    m_detector->detect(image, keypoints);

    if (keypoints.empty())
        return false;
    
    m_extractor->compute(image, keypoints, descriptors);
    if (keypoints.empty())
        return false;

    return true;
}

bool Tracker::extractFeaturesMasked(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const
{
    assert(!image.empty());
    assert(image.channels() == 1);
    
    m_detector->detect(image, keypoints, m_trackingInfo.maskROI);
    
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
        //m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);
        m_matcher->knnMatch(queryDescriptors, m_imageTarget.descriptors, m_knnMatches, 2);

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
    
//    stringstream ss;
//    ss << "found " << matches.size() << " matches";
//    Debug::Log(ss, Color::Yellow);

}

void Tracker::getGMSMatches(Size& size1, Size& size2, std::vector<cv::KeyPoint>& kpts1, std::vector<cv::KeyPoint>& kpts2, cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches)
{
    matches.clear();
    //m_knnMatches.clear();
    std::vector<cv::DMatch> matchesGMS;

    m_matcher->match(queryDescriptors, matches);
    cv::xfeatures2d::matchGMS(size1,
                              size2,
                              kpts1,
                              kpts2,
                              matches,
                              matchesGMS,
                              true,
                              true,
                              6.0);
    
    matches.swap(matchesGMS);
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
    //const int minNumberMatchesAllowed = 4;

    if (matches.size() < 3)
    {
        minNumberMatchesAllowed = 3;
        return false;
    }
    
    
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
                                    inliersMask,
                                    1000,
                                    0.965);

    std::vector<cv::DMatch> inliers;
    for (size_t i=0; i<inliersMask.size(); i++)
    {
        if (inliersMask[i])
            inliers.push_back(matches[i]);
    }

    matches.swap(inliers);
    
    if(matches.size() > 4)
    {
        minNumberMatchesAllowed = matches.size() - 1;
    }
    
    return matches.size() > minNumberMatchesAllowed;
}

cv::Mat Tracker::createMaskROI()
{
    Size size = Size(m_grayImg.cols, m_grayImg.rows);
    Mat mask = Mat::zeros(size, CV_8UC1);
    
    //quad mask
    vector<Point> ROI_Poly;
    approxPolyDP(m_trackingInfo.raw_points2d, ROI_Poly, 1.0, true);
    fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0);
    
    //dots mask
//    for(int i = 0; i < m_matches.size(); i++)
//    {
//        Point p = m_queryKeypoints[ m_matches[i].queryIdx].pt;
//        cv::circle(mask, p, 3, Scalar(255, 255, 255), CV_FILLED, 8, 0);
//    }
    
    return mask;
}

cv::Mat Tracker::resetMaskROI()
{
    Size size = Size(m_grayImg.cols, m_grayImg.rows);
    Mat mask = Mat::zeros(size, CV_8UC1);
    mask = cv::Scalar(255, 255, 255);
    return mask;
}
