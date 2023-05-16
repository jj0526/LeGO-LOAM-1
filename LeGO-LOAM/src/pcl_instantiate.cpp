#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

typedef pcl::PointCloud<PointXYZIGround> PointCloudXYZIGround;

template class PointCloudXYZIGround;

template void pcl::removeNaNFromPointCloud<PointXYZIGround>(const PointCloudXYZIGround&, PointCloudXYZIGround&, std::vector<int>&);
