#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include "cloud_msgs/cloud_info.h"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
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

#define PI 3.14159265

using namespace std;

// Define the custom point type with the "isGround" field
struct PointXYZIRGround : public pcl::PointXYZIR
{
  int isGround;  // Additional "isGround" field
};

typedef PointXYZIRGround PointType;

extern const string pointCloudTopic;
extern const string imuTopic;
extern const string fileDirectory;
extern const bool useCloudRing;
extern const int N_SCAN;
extern const int Horizon_SCAN;
extern const float ang_res_x;
extern const float ang_res_y;
extern const float ang_bottom;
extern const int groundScanInd;
extern const bool loopClosureEnableFlag;
extern const double mappingProcessInterval;
extern const float scanPeriod;
extern const int systemDelay;
extern const int imuQueLength;
extern const float sensorMinimumRange;
extern const float sensorMountAngle;
extern const float segmentTheta;
extern const int segmentValidPointNum;
extern const int segmentValidLineNum;
extern const float segmentAlphaX;
extern const float segmentAlphaY;
extern const int edgeFeatureNum;
extern const int surfFeatureNum;
extern const int sectionsTotal;
extern const float edgeThreshold;
extern const float surfThreshold;
extern const float nearestFeatureSearchSqDist;
extern const float surroundingKeyframeSearchRadius;
extern const int surroundingKeyframeSearchNum;
extern const float historyKeyframeSearchRadius;
extern const int historyKeyframeSearchNum;
extern const float historyKeyframeFitnessScore;
extern const float globalMapVisualizationSearchRadius;

struct smoothness_t
{
  float value;
  size_t ind;
};

struct by_value
{
  bool operator()(smoothness_t const &left, smoothness_t const &right)
  {
    return left.value < right.value;
  }
};

// Point cloud type with "ring" channel
struct PointXYZIR
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  int isGround;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x,)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
)

// Point cloud type with 6D pose info
struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  int isGround;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, roll, roll)
                                  (float, pitch, pitch)
                                  (float, yaw, yaw)
                                  (double, time, time)
)

typedef PointXYZIRPYT PointTypePose;

#endif
