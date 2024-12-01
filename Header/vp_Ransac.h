#ifndef VP_RANSAC_H
#define VP_RANSAC_H

#include "general_include.h"
#include "functions.h"
#include <random>


// a Ransac method for VP finding
class VP_ransac
{
public:
    std::vector<float> weights;

    VP_ransac() {}
    VP_ransac(vector<cv::Vec4f> & line_parameters, std::vector<cv::Vec3f>& inliers, const cv::Mat & img, string& img_name);
    void Ransac_implement(std::vector<cv::Vec3f>& inliers, std::vector<cv::Vec3f>& line_parameters, float theshold, int max_i, const cv::Mat & img);
};


// Function to calculate the angle between two vectors
float calculateAngle(const std::pair<float, float>& vector1, const std::pair<float, float>& vector2) {
    float dotProduct = vector1.first * vector2.first + vector1.second * vector2.second;
    float magnitude1 = std::sqrt(vector1.first * vector1.first + vector1.second * vector1.second);
    float magnitude2 = std::sqrt(vector2.first * vector2.first + vector2.second * vector2.second);
    float angle = std::acos(dotProduct / (magnitude1 * magnitude2));
    return angle;
}


// Function to find the knee point in a curve
int detectKneePoint(const std::vector<float>& x, const std::vector<int>& y) {
    int kneePointIndex = 0;
    float maxAngle = -1.0;

    for (size_t i = 1; i < x.size() - 1; ++i) {
        std::pair<float, float> vector1 = {x[i] - x[i - 1], y[i] - y[i - 1]};
        std::pair<float, float> vector2 = {x[i + 1] - x[i], y[i + 1] - y[i]};
        float angle = calculateAngle(vector1, vector2);

        if (angle > maxAngle) {
            maxAngle = angle;
            kneePointIndex = i;
        }
    }

    return kneePointIndex;
}


// Function to calculate the angle between two lines given in <a, b, c> representation
float angleBetweenLines(const cv::Vec3f& line1, const cv::Vec3f& line2) {
    // Compute the direction vectors of each line
    cv::Vec2f dir1(-line1[1], line1[0]); // Direction vector of line1 perpendicular to <a, b>
    cv::Vec2f dir2(-line2[1], line2[0]); // Direction vector of line2 perpendicular to <a, b>
    
    // Normalize the direction vectors
    dir1 = dir1 / cv::norm(dir1);
    dir2 = dir2 / cv::norm(dir2);
    
    // Compute the cosine of the angle between the two lines
    float dot = dir1.dot(dir2);
    // Clamp dot to avoid numerical issues with acos
    dot = std::max(-1.0f, std::min(1.0f, dot));
    
    // Compute the angle between the two lines
    float angle = std::acos(dot);
    
    // Make sure the angle is between 0 and pi/2
    angle = std::min(angle, static_cast<float>(CV_PI) - angle);
    
    return angle;
}


// Function to initialize weights based on the angle between lines
void initializeWeights(const std::vector<cv::Vec3f>& lines, std::vector<float>& weights){
    size_t numLines = lines.size();
    weights.resize(numLines, 0.0f);

    // Initialize weights
    for (size_t i = 0; i < numLines; ++i) {
        for (size_t j = i + 1; j < numLines; ++j) {
            // Calculate the angle between line i and line j
            float angle = angleBetweenLines(lines[i], lines[j]);
            
            // Use the angle to determine the weight, smaller angle gets higher weight
            float weight = std::exp(-angle); // Example: using exponential decay
            
            // Add the weight contribution to both lines
            weights[i] += weight;
            weights[j] += weight;
        }
    }

    // Normalize weights to sum to 1, if desired
    float totalWeight = std::accumulate(weights.begin(), weights.end(), 0.0f);
    for (auto& w : weights) {
        w /= totalWeight;
    }
}


// Function to estimate the vanishing point from two lines in the <a, b, c> form
cv::Point2f findIntersectionPoint(const cv::Vec3f& line1, const cv::Vec3f& line2) {
    cv::Mat A = (cv::Mat_<float>(2, 2) << line1[0], line1[1],
                                             line2[0], line2[1]);
    cv::Mat b = (cv::Mat_<float>(2, 1) << -line1[2],
                                             -line2[2]);
    cv::Mat x;
    cv::solve(A, b, x, cv::DECOMP_SVD); // Use SVD to solve for robustness

    return cv::Point2f(x.at<float>(0, 0), x.at<float>(1, 0));
}


// Function to check if a line is an inlier given a vanishing point and a threshold
bool isLineAnInlier(const cv::Vec3f& line, const cv::Point2f& vanishingPoint, float threshold) {
    // The line equation is a*x + b*y + c = 0
    // The distance of the point (x0, y0) from the line is given by |a*x0 + b*y0 + c| / sqrt(a^2 + b^2)
    float distance = std::abs(line[0] * vanishingPoint.x + line[1] * vanishingPoint.y + line[2]) /
                     std::sqrt(line[0] * line[0] + line[1] * line[1]);

    // If the distance is less than the threshold, the point is an inlier
    return distance < threshold;
}


// Constructor for the VP_ransac class
VP_ransac::VP_ransac(std::vector<cv::Vec4f>& line_parameters, std::vector<cv::Vec3f>& inliers, const cv::Mat & img, string& img_name)
{
    std::cout << "\nPerforming Weighted Ransac..." << std::endl;

    // float threshold = 10;       // the distance between vp candidates and lines candidates
    int max_iteration = 200;       // maximum number of iteration


    vector<cv::Vec3f> line_paras;

    for(int i = 0; i<line_parameters.size(); i++){
        cv::Vec3f lpara;
        lpara[0] = line_parameters[i][1];
        lpara[1] = -line_parameters[i][0];
        lpara[2] = line_parameters[i][0]*line_parameters[i][3] - line_parameters[i][1]*line_parameters[i][2];

        line_paras.push_back(lpara);
    }


    std::vector<int> inlierCounts;
    std::vector<cv::Vec3f> temp_inliers;
    std::vector<float> thresholds = {2, 3, 5, 8, 13, 21, 34, 55, 89};

    // Initialize the weights for the lines 
    for (float threshold : thresholds) {
        initializeWeights(line_paras, weights); 
        Ransac_implement(temp_inliers, line_paras, threshold, max_iteration, img);
        if (!temp_inliers.empty()) {
            inlierCounts.push_back(temp_inliers.size());
            temp_inliers.clear();
        }
    }

    float optimalThreshold = 10;
    if (!inlierCounts.empty()){
        // Assuming you have implemented or used a library for detectKneePoint
        int kneeIndex = detectKneePoint(thresholds, inlierCounts);
        optimalThreshold = thresholds[kneeIndex];
    }
    initializeWeights(line_paras, weights);  
    Ransac_implement(inliers, line_paras, 10, max_iteration, img);
}


// Function to implement the RANSAC algorithm to get inliers
void VP_ransac::Ransac_implement(std::vector<cv::Vec3f>& inliers, std::vector<cv::Vec3f>& line_parameters, float theshold, int max_i, const cv::Mat & img)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<float> updatedWeights = weights;
    std::discrete_distribution<> dist(updatedWeights.begin(), updatedWeights.end());

    float inlierIncreaseFactor = 1.2f; // Increase factor for inliers
    float outlierDecreaseFactor = 0.8f; // Decrease factor for outliers

    for (int i = 0; i < max_i; ++i) {
        // Randomly select two lines based on their weights
        int idx1 = dist(gen);
        int idx2 = dist(gen);
        if (idx1 == idx2) continue; // Ensure they are different lines

        // Estimate vanishing point using the selected lines
        cv::Point2f vp = findIntersectionPoint(line_parameters[idx1], line_parameters[idx2]);

        // Determine inliers for the estimated vanishing point
        std::vector<cv::Vec3f> temp_inliers;
        for (size_t j = 0; j < line_parameters.size(); ++j) {
            if (isLineAnInlier(line_parameters[j], vp, theshold)) {
                temp_inliers.push_back(line_parameters[j]);
                updatedWeights[j] *= inlierIncreaseFactor; // Increase weight for inliers
            }
            else {
                updatedWeights[j] *= outlierDecreaseFactor; // Decrease weight for outliers
            }
        }

        // Normalize the updated weights
        float weightSum = std::accumulate(updatedWeights.begin(), updatedWeights.end(), 0.0f);
        for (auto& w : updatedWeights) {
            w /= weightSum;
        }

        weights = updatedWeights;
        // If the current set of inliers is the best so far, update the best model
        if (temp_inliers.size() > inliers.size()) {
            inliers = temp_inliers;
        }
    }
}


#endif // VP_RANSAC_H
