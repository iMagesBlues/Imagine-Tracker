//
//  ARUtils.cpp
//  imagineAR
//
//  Created by Ronnie Miguel Besas on 6/3/18.
//

#include "ARUtils.hpp"

void ARUtils::GetGray(cv::Mat mat, cv::Mat &outMat)
{
    if (mat.channels()  == 3)
        cv::cvtColor(mat, outMat, CV_BGR2GRAY);
    else if (mat.channels() == 4)
        cv::cvtColor(mat, outMat, CV_BGRA2GRAY);
    else if (mat.channels() == 1)
        outMat = mat;
    
    //effective but slow
    cv::Mat outMat2;
    cv::GaussianBlur(outMat, outMat2, cv::Size(0, 0), 3);
    cv::addWeighted(outMat, 4, outMat2, -3, 0, outMat2);
    
    outMat = outMat2.clone();
}

void ARUtils::Resize(cv::Mat mat, cv::Mat &outMat, int maxside)
{
    cv::Size scaled = GetScaledSize(cv::Size(mat.cols, mat.rows), maxside);
    cv::resize(mat, outMat, scaled);
}

cv::Size ARUtils::GetScaledSize(cv::Size size, int maxside)
{
    int longS, shortW, shortH;
    
    longS = std::max(size.width, size.height);
    shortW = size.width  * (float)(maxside)  / longS;
    shortH = size.height * (float)(maxside) / longS;
    
    return cv::Size(shortW, shortH);
}
