#pragma once
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "picoflexx.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>

class PointCloudUtils {
public:
    PointCloudUtils() = default;

    // Fast -90° rotation around Z-axis: (x,y,z) → (y,-x,z)
    static void rotateZ90(std::vector<point3d>& cloud);

    pcl::PointCloud<pcl::PointXYZ> convertToPCL(const std::vector<point3d>& cloud) const;
    pcl::PointCloud<pcl::PointXYZ> downsamplePCL(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, float downsampled_size) const;
    void visualizePCL(const pcl::PointCloud<pcl::PointXYZ>& pcl_cloud) const;
    void detectCylinder(pcl::PointCloud<pcl::PointXYZ>& cloud) const;
    void detectCircle(pcl::PointCloud<pcl::PointXYZ>& cloud) const;
    pcl::PolygonMesh greedyTriangulation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud) const;
    void visualizeMesh(const pcl::PolygonMesh& mesh) const;
    std::vector<pcl::PointIndices> regionGrowingSegmentation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud) const;

};