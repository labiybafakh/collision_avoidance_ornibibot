#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include <vector>
#include <atomic>
#include <memory>
#include <array>
#include <cstdint>
#include <limits>
#include <opencv2/opencv.hpp>
#include "picoflexx.hpp"

struct ObstacleAvoidanceParams {
    std::atomic<float> safetyDistance{2.0f};     // meters
    std::atomic<float> turnAngle{30.0f};         // degrees
    std::atomic<float> maxRange{4.0f};           // meters - max detection range
    std::atomic<float> minRange{0.2f};           // meters - min detection range  
    std::atomic<float> maxXDistance{2.0f};       // meters - navigation corridor width (±2m)
    std::atomic<float> maxYDistance{0.1f};       // meters - navigation height band (±1m)
    std::atomic<bool> enabled{true};
};

enum class AvoidanceCommand {
    FORWARD = 0,
    TURN_LEFT = -1,
    TURN_RIGHT = 1,
    STOP = 2
};

class ObstacleAvoidance {
public:
    ObstacleAvoidance();
    ~ObstacleAvoidance() = default;
    
    // Process point cloud and return avoidance command
    AvoidanceCommand processPointCloud(const std::vector<point3d>& cloud);
    
    // Process depth image directly and return avoidance command
    AvoidanceCommand processDepthImage(const cv::Mat& depthImage);
    
    // Get current parameters
    ObstacleAvoidanceParams& getParams() { return params_; }
    
    // Get obstacle detection statistics
    struct Statistics {
        size_t totalPoints;
        size_t obstaclePoints;
        size_t leftObstacles;
        size_t rightObstacles;
        float minDistance;
        AvoidanceCommand lastCommand;
    };
    
    const Statistics& getStats() const { return stats_; }
    
    // Latest obstacle grid visualization
    cv::Mat getGridVisualization() const;

private:
    // Grid-based obstacle detection (similar to your 8x8 matrix approach)
    static constexpr size_t GRID_WIDTH = 480;
    static constexpr size_t GRID_HEIGHT = 640;

    ObstacleAvoidanceParams params_;
    Statistics stats_;
    std::array<std::array<uint8_t, GRID_WIDTH>, GRID_HEIGHT> lastGrid_{};
    
    // Convert 3D points to 2D grid for obstacle detection
    void pointCloudToGrid(const std::vector<point3d>& cloud, 
                         bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH]);
    
    // Convert depth image directly to obstacle grid (Eq. obstacle-matrix)
    void depthImageToGrid(const cv::Mat& depthImage, 
                         bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH]);
    
    // Analyze grid for obstacle patterns
    AvoidanceCommand analyzeObstacleGrid(const bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH]);
};

#endif
