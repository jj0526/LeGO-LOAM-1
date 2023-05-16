#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "utility.h"

using PointCloudXYZIGround = pcl::PointCloud<PointXYZIGround>;

template class pcl::PointCloud<PointXYZIGround>;

template void removeNaNFromPointCloud<PointXYZIGround>(const PointCloudXYZIGround&, PointCloudXYZIGround&, std::vector<int>&);
