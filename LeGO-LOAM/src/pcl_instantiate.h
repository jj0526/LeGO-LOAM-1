#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "utility.h"

using PointCloudXYZIGround = pcl::PointCloud<PointXYZIGround>;

extern template void removeNaNFromPointCloud<PointXYZIGround>(
    const PointCloudXYZIGround&, PointCloudXYZIGround&, std::vector<int>&);
