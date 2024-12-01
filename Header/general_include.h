#ifndef GENERAL_INCLUDE_H
#define GENERAL_INCLUDE_H

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>

//boost
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;


using namespace std;
using namespace chrono;

#endif
