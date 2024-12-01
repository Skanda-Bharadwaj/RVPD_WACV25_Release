#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "general_include.h"

static float resizeRatio = 1.0f;

// sort Sift feature by the size of features
bool SIFTfunction (cv::KeyPoint i,cv::KeyPoint j) { return (i.size>j.size); }

// return the Eucidean Distance between features
float EuclideanDistance(cv::KeyPoint p1, cv::KeyPoint p2)
{
    return sqrt((p1.pt.x - p2.pt.x)*(p1.pt.x - p2.pt.x) + (p1.pt.y - p2.pt.y)*(p1.pt.y - p2.pt.y));
}

// compute the Eucidean Distance between features
bool EuclideanDist(cv::KeyPoint p1, cv::KeyPoint p2)
{
    float distance = sqrt((p1.pt.x - p2.pt.x)*(p1.pt.x - p2.pt.x) + (p1.pt.y - p2.pt.y)*(p1.pt.y - p2.pt.y));
    return distance >= 10;
}

// compute the angle difference
bool angleDiff(cv::KeyPoint p1, cv::KeyPoint p2)
{
    float diff = abs(p1.angle - p2.angle);
    return diff <= 30;
}

// compute the angle difference
float scaleDiff(cv::KeyPoint p1, cv::KeyPoint p2)
{
    return (p1.size / p2.size) > 1 ? (p1.size / p2.size)  : (p2.size / p1.size) ;
}

// draw the sift feature
void drawFeatures(cv:: Mat & img, vector<cv::KeyPoint> & points, cv::Scalar color, bool singlePrint = false, double radiusFactor = 2)
{
    for (int i = 0; i< points.size(); i++)
    {
        double radius = points[i].size*radiusFactor;
        double Pi = 3.1415926;
        circle(img, points[i].pt, radius,  color, 2, 8);
        line(img, points[i].pt, cv::Point(points[i].pt.x+radius*cos(points[i].angle/180*Pi), points[i].pt.y+radius*sin(points[i].angle/180*Pi))
             , color, 2);
        if(singlePrint)
        {
            cv::imshow("SIFT Features", img);
            cv::waitKey(200);
        }
    }
   // cv::imshow("SIFT Features", img);
    //cv::waitKey(10);
}

// compute the displace Vector
pair<float, float> displaceVector(cv::KeyPoint p1, cv::KeyPoint p2)
{
    if(p1.size > p2.size)
    {
        return make_pair(p1.pt.x-p2.pt.x, p1.pt.y- p2.pt.y);
    }
    else
    {
        return make_pair(p2.pt.x-p1.pt.x, p2.pt.y- p1.pt.y);
    }
}



// define the linearity difference

bool displaceXSort (cv::KeyPoint i, cv::KeyPoint j) { return (i.pt.x>j.pt.x); }
bool displaceYSort (cv::KeyPoint i, cv::KeyPoint j) { return (i.pt.y>j.pt.y); }
bool displaceScaleSort (cv::KeyPoint i, cv::KeyPoint j)
{
    if (i.size == j.size)
        return (i.pt.x>j.pt.x);
    else
        return (i.size>j.size);
}

// // Show the vanishing point in the image
void vp_imshow(cv::Mat& originalImage, const std::vector<cv::Vec3f>& inliers, const cv::Point2f& vp, const float resizeRatio, const std::string& vp_img_path){

    cv::Point2f vanishingPoint = cv::Point2f(vp.x * resizeRatio, vp.y * resizeRatio);
    int offset = 100;
    int x_add = 0, y_add = 0; // Offsets to shift the original image within the new canvas

    // Determine the new canvas size and the offset to place the original image
    int vp_img_cols = originalImage.cols, vp_img_rows = originalImage.rows;

    if (vanishingPoint.x < 0) {
        x_add = std::abs(vanishingPoint.x) + offset; // Expand leftwards with offset
        vp_img_cols += x_add;
    } else if (vanishingPoint.x >= originalImage.cols) {
        vp_img_cols += vanishingPoint.x + offset; // Expand rightwards with offset
    }

    if (vanishingPoint.y < 0) {
        y_add = std::abs(vanishingPoint.y) + offset; // Expand upwards with offset
        vp_img_rows += y_add;
    } else if (vanishingPoint.y >= originalImage.rows) {
        vp_img_rows += vanishingPoint.y + offset; // Expand downwards with offset
    }

    // Create a new canvas if necessary and place the original image accordingly
    cv::Mat img_copy = originalImage.clone();
    cv::Mat vp_img(vp_img_rows, vp_img_cols, CV_8UC3, cv::Scalar(255,255,255));
    cv::Rect roi_rect = cv::Rect(x_add, y_add, originalImage.cols, originalImage.rows);
    img_copy.copyTo(vp_img(roi_rect));

    // Adjust the vanishing point position based on the new canvas
    cv::Point2f adjustedVP = vanishingPoint + cv::Point2f(x_add, y_add);

    // Draw the lines extending towards the vanishing point
    for (const auto& line : inliers) {
        // Calculate two points on the line that are within the canvas
        float a = line[0], b = line[1], c = line[2] + a * (-x_add) + b * (-y_add);; // Adjust 'c' based on offset
        cv::Point2f pt1, pt2;
        if (std::fabs(b) > 1e-6) {
            pt1.x = 0;
            pt1.y = -(c + a * pt1.x) / b;
            pt2.x = vp_img.cols;
            pt2.y = -(c + a * pt2.x) / b;
        } else {
            pt1.y = 0;
            pt1.x = -(c + b * pt1.y) / a;
            pt2.y = vp_img.rows;
            pt2.x = -(c + b * pt2.y) / a;
        }

        // Adjust points based on the offset
        // pt1.x += x_add;
        // pt1.y += y_add;
        // pt2.x += x_add;
        // pt2.y += y_add;

        // Pick a random color for the line
        cv::Scalar color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
        cv::line(vp_img, pt1, pt2, color, 2);
    }

    // Draw the vanishing point
    cv::circle(vp_img, adjustedVP, 10, cv::Scalar(0, 0, 255), -1);

    cv::imwrite(vp_img_path, vp_img);
    
}


void writeToCSV(const std::vector<float>& thresholds, const std::vector<int>& inlierCounts, int kneeIndex, const std::string& csv_path, const std::string& img_name) {
    string img_name_wo_ext = img_name.substr(0, img_name.find('.'));
    string filename = csv_path + img_name_wo_ext + ".csv";
    
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    // Write headers
    file << "Threshold,InlierCount,KneePoint\n";

    // Write data
    for (size_t i = 0; i < thresholds.size(); ++i) {
        file << thresholds[i] << "," << inlierCounts[i] << ",";
        // Mark the knee point with a 1, others with a 0
        if (i == kneeIndex) {
            file << "1\n";
        } else {
            file << "0\n";
        }
    }

    file.close();
    std::cout << "Data written to " << filename << std::endl;
}


#endif // FUNCTIONS_H
