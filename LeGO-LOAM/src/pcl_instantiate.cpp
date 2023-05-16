#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

template class PointCloud<PointXYZIGround>;

template void pcl::removeNaNFromPointCloud<PointXYZIGround>(const PointCloud<PointXYZIGround>&, PointCloud<PointXYZIGround>&, std::vector<int>&);