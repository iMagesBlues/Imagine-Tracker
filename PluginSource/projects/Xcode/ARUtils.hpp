//
//  ARUtils.hpp
//  imagineAR
//
//  Created by Ronnie Miguel Besas on 6/3/18.
//

#ifndef ARUtils_hpp
#define ARUtils_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>

class ARUtils
{
public:
    static const int MAX_IMAGE_SIDE = 350;
    
    static void     GetGray (cv::Mat mat, cv::Mat &outMat);
    static void     Resize  (cv::Mat mat, cv::Mat &outMat, int maxside = ARUtils::MAX_IMAGE_SIDE);
    
    static cv::Size GetScaledSize (cv::Size size, int maxside = ARUtils::MAX_IMAGE_SIDE);
};

#endif /* ARUtils_hpp */
