#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>


#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


#include "pointmatcher/Timer.h"
#include "pointmatcher/PointMatcher.h"

#include "nabo/nabo.h"

#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/point_cloud.h"

#define PI 3.14159265

using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Matches Matches;

typedef typename Nabo::NearestNeighbourSearch<float> NNS;
typedef typename NNS::SearchType NNSearchType;

#endif
