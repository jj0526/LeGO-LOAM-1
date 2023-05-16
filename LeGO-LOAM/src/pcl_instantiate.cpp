#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "utility.h"

template class pcl::PointCloud<PointXYZIGround>;  // Move this line before the class definition

using PointCloudXYZIGround = pcl::PointCloud<PointXYZIGround>;

template void removeNaNFromPointCloud<PointXYZIGround>(const PointCloudXYZIGround&, PointCloudXYZIGround&, std::vector<int>&);
