#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "utility.h"

typedef pcl::PointCloud<PointXYZIGround> PointCloudXYZIGround;

template class PointCloudXYZIGround;

template void pcl::removeNaNFromPointCloud<PointType>(const PointCloudXYZIGround&, PointCloudXYZIGround&, std::vector<int>&);
