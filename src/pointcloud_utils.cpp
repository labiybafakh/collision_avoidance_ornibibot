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

void PointCloudUtils::detectCylinder(pcl::PointCloud<pcl::PointXYZ>& cloud) const {
    std::cout << "Cylinder detection disabled (RANSAC removed)" << std::endl;
}

void PointCloudUtils::detectCircle(pcl::PointCloud<pcl::PointXYZ>& cloud) const {
    std::cout << "Circle detection disabled (RANSAC removed)" << std::endl;
}

pcl::PolygonMesh PointCloudUtils::greedyTriangulation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud) const {
    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(input_cloud.makeShared());
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);

    // Concatenate XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(input_cloud, *normals, *cloud_with_normals);

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the parameters
    gp3.setSearchRadius(0.025); // Adjust as needed
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18); // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    return triangles;
}

void PointCloudUtils::visualizeMesh(const pcl::PolygonMesh& mesh) const {
    std::cout << "Mesh visualization disabled (no OpenGL in processor)" << std::endl;
    std::cout << "Mesh has " << mesh.polygons.size() << " polygons" << std::endl;
}

std::vector<pcl::PointIndices> PointCloudUtils::regionGrowingSegmentation(const pcl::PointCloud<pcl::PointXYZ>& input_cloud) const {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_cloud.makeShared());
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);

    // Region growing segmentation
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(input_cloud.makeShared());
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); // 3 degrees
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    return clusters;
}