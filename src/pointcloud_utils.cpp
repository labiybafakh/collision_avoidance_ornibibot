#include "pointcloud_utils.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

void PointCloudUtils::rotateZ90(std::vector<point3d>& cloud) {
    for (auto& pt : cloud) {
        float temp = pt.x;
        pt.x = pt.y;
        pt.y = -temp;
    }
}

pcl::PointCloud<pcl::PointXYZ> PointCloudUtils::convertToPCL(const std::vector<point3d>& cloud) const {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    for (const auto& pt : cloud) {
        pcl_cloud.points.emplace_back(pt.x, pt.y, pt.z);
    }
    pcl_cloud.width = pcl_cloud.points.size();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false;
    return pcl_cloud;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudUtils::downsamplePCL(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, float downsampled_size) const {
    pcl::PointCloud<pcl::PointXYZ> down_sampled;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud.makeShared());
    sor.setLeafSize(downsampled_size, downsampled_size, downsampled_size);
    sor.filter(down_sampled);
    return down_sampled;
}

void PointCloudUtils::visualizePCL(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud) const {
    std::cout << "Visualization disabled (no OpenGL in processor)" << std::endl;
}