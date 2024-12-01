#ifndef VP_COMPUTE_H
#define VP_COMPUTE_H

#include <general_include.h>
#include "functions.h"
#include <vp_Ransac.h>

cv::Point2f compute_VP_candidate(std::vector<cv::Vec3f> inliers)
{
    // Construct matrix A from line coefficients
    cv::Mat A(inliers.size(), 3, CV_32F);
    for (size_t i = 0; i < inliers.size(); ++i) {
        A.at<float>(i, 0) = inliers[i][0]; // a
        A.at<float>(i, 1) = inliers[i][1]; // b
        A.at<float>(i, 2) = inliers[i][2]; // c
    }

    // Apply SVD
    cv::Mat w, u, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::FULL_UV);

    // The vanishing point is the last column of V (or vt transposed)
    cv::Mat V = vt.t(); // Transpose vt to get V
    cv::Mat vp = V.col(V.cols - 1); // Last column

    // Normalize the vanishing point to convert from homogeneous to Cartesian coordinates
    cv::Point2f vp_candidate(vp.at<float>(0) / vp.at<float>(2), vp.at<float>(1) / vp.at<float>(2));

    return vp_candidate;
}


#endif // VP_COMPUTE_H
