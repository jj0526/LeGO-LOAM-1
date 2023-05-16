#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

template class pcl::PointCloud<pcl::PointXYZIGround>;

template void pcl::removeNaNFromPointCloud<pcl::PointXYZIGround>(const pcl::PointCloud<pcl::PointXYZIGround>&, pcl::PointCloud<pcl::PointXYZIGround>&, std::vector<int>&);