#include "obstacle_avoidance.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstring>

ObstacleAvoidance::ObstacleAvoidance() {
    // Initialize statistics
    stats_.totalPoints = 0;
    stats_.obstaclePoints = 0;
    stats_.leftObstacles = 0;
    stats_.rightObstacles = 0;
    stats_.minDistance = 0.0f;
    stats_.lastCommand = AvoidanceCommand::FORWARD;
    for (auto &row : lastGrid_) {
        row.fill(0);
    }
}

AvoidanceCommand ObstacleAvoidance::processPointCloud(const std::vector<point3d>& cloud) {
    if (!params_.enabled || cloud.empty()) {
        stats_.lastCommand = AvoidanceCommand::FORWARD;
        return AvoidanceCommand::FORWARD;
    }
    
    // Reset statistics
    stats_.totalPoints = cloud.size();
    stats_.obstaclePoints = 0;
    stats_.leftObstacles = 0;
    stats_.rightObstacles = 0;
    stats_.minDistance = std::numeric_limits<float>::max();
    
    // Create obstacle grid
    bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH] = {false};
    
    pointCloudToGrid(cloud, obstacleGrid);
    
    // Analyze grid for navigation commands
    AvoidanceCommand command = analyzeObstacleGrid(obstacleGrid);
    
    stats_.lastCommand = command;
    return command;
}

AvoidanceCommand ObstacleAvoidance::processDepthImage(const cv::Mat& depthImage) {
    if (!params_.enabled || depthImage.empty()) {
        stats_.lastCommand = AvoidanceCommand::FORWARD;
        return AvoidanceCommand::FORWARD;
    }
    
    // Reset statistics
    stats_.totalPoints = depthImage.rows * depthImage.cols;
    stats_.obstaclePoints = 0;
    stats_.leftObstacles = 0;
    stats_.rightObstacles = 0;
    stats_.minDistance = std::numeric_limits<float>::max();
    
    // Create obstacle grid
    bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH] = {false};
    
    depthImageToGrid(depthImage, obstacleGrid);
    
    // Analyze grid for navigation commands
    AvoidanceCommand command = analyzeObstacleGrid(obstacleGrid);
    
    stats_.lastCommand = command;
    return command;
}

void ObstacleAvoidance::pointCloudToGrid(const std::vector<point3d>& cloud, 
                                        bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH]) {
    
    // Clear last grid visualization buffer
    for (auto &row : lastGrid_) {
        row.fill(0);
    }
    
    // Use configurable navigation envelope parameters
    const float MAX_RANGE = params_.maxRange;
    const float MIN_RANGE = paramjs_.minRange;
    const float MAX_X_DISTANCE = params_.maxXDistance;
    const float MAX_Y_DISTANCE = params_.maxYDistance;
    
    for (const auto& point : cloud) {
        // Calculate distance from camera
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        
        // Update minimum distance
        stats_.minDistance = std::min(stats_.minDistance, distance);
        
        // Check if point is within navigation envelope (distance-based filtering)
        if (point.z >= MIN_RANGE && point.z < MAX_RANGE &&
            abs(point.x) <= MAX_X_DISTANCE && abs(point.y) <= MAX_Y_DISTANCE) {
            
            // Process points within navigation corridor
            if (distance < params_.safetyDistance) {
                stats_.obstaclePoints++;
            }
            
            // Map 3D coordinates directly to grid (distance-based, not angle-based)
            // X mapping: -MAX_X_DISTANCE to +MAX_X_DISTANCE -> 0 to GRID_WIDTH-1
            int gridX = static_cast<int>((point.x + MAX_X_DISTANCE) * GRID_WIDTH / (2 * MAX_X_DISTANCE));
            // Y mapping: -MAX_Y_DISTANCE to +MAX_Y_DISTANCE -> 0 to GRID_HEIGHT-1  
            int gridY = static_cast<int>((point.y + MAX_Y_DISTANCE) * GRID_HEIGHT / (2 * MAX_Y_DISTANCE));
                
            // Clamp to grid bounds
            gridX = std::max(0, std::min(gridX, static_cast<int>(GRID_WIDTH) - 1));
            gridY = std::max(0, std::min(gridY, static_cast<int>(GRID_HEIGHT) - 1));
            
            // Mark obstacle in grid if within safety distance
            if (distance < params_.safetyDistance) {
                obstacleGrid[gridY][gridX] = true;
                lastGrid_[gridY][gridX] = 255;
                
                // Count left vs right obstacles (based on X coordinate)
                if (point.x < 0) {  // Left side (negative X)
                    stats_.leftObstacles++;
                } else {            // Right side (positive X)
                    stats_.rightObstacles++;
                }
            }
        }
    }

}

void ObstacleAvoidance::depthImageToGrid(const cv::Mat& depthImage, 
                                        bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH]) {
    
    // Clear last grid visualization buffer
    for (auto &row : lastGrid_) {
        row.fill(0);
    }
    
    const float d_safe = params_.safetyDistance;  // d_safe from your equation
    const int halfWidth = GRID_WIDTH / 2;
    
    // Resize depth image to match our grid dimensions if needed
    cv::Mat resizedDepth;
    if (depthImage.rows != GRID_HEIGHT || depthImage.cols != GRID_WIDTH) {
        cv::resize(depthImage, resizedDepth, cv::Size(GRID_WIDTH, GRID_HEIGHT), 0, 0, cv::INTER_LINEAR);
    } else {
        resizedDepth = depthImage;
    }
    
    // Direct implementation of Eq. obstacle-matrix: O_ij = 1 if D_ij < d_safe, 0 otherwise
    for (int i = 0; i < GRID_HEIGHT; i++) {
        for (int j = 0; j < GRID_WIDTH; j++) {
            float D_ij = resizedDepth.at<float>(i, j);  // Distance at pixel (i,j)
            
            // Skip invalid/zero depth values
            if (D_ij <= 0 || std::isnan(D_ij) || std::isinf(D_ij)) {
                continue;
            }
            
            // Update minimum distance
            stats_.minDistance = std::min(stats_.minDistance, D_ij);
            
            // Apply obstacle matrix equation: O_ij = 1 if D_ij < d_safe
            if (D_ij < d_safe) {
                obstacleGrid[i][j] = true;  // O_ij = 1
                lastGrid_[i][j] = 255;      // For visualization
                stats_.obstaclePoints++;
                
                // Count left vs right obstacles for Eq. left-count and right-count
                if (j < halfWidth) {        // Left side: j = 1 to n/2
                    stats_.leftObstacles++;  // C_left += O_ij
                } else {                    // Right side: j = n/2+1 to n  
                    stats_.rightObstacles++; // C_right += O_ij
                }
            }
            // else: O_ij = 0 (already initialized to false)
        }
    }
}

AvoidanceCommand ObstacleAvoidance::analyzeObstacleGrid(const bool obstacleGrid[GRID_HEIGHT][GRID_WIDTH]) {
    
    // Count total obstacles in grid
    size_t totalGridObstacles = 0;
    for (size_t y = 0; y < GRID_HEIGHT; y++) {
        for (size_t x = 0; x < GRID_WIDTH; x++) {
            if (obstacleGrid[y][x]) {
                totalGridObstacles++;
            }
        }
    }
    
    // No obstacles detected
    if (totalGridObstacles == 0) {
        return AvoidanceCommand::FORWARD;
    }
    
    // Critical obstacle density - stop
    // if (totalGridObstacles > (GRID_WIDTH * GRID_HEIGHT) * 0.6f) {
    //     return AvoidanceCommand::STOP;
    // }
    
    // Determine turn direction based on obstacle distribution
    // (Similar to your left/right counting approach)
    if (stats_.leftObstacles > stats_.rightObstacles) {
        return AvoidanceCommand::TURN_RIGHT;
    } else if (stats_.rightObstacles > stats_.leftObstacles) {
        return AvoidanceCommand::TURN_LEFT;
    }
    
    // Equal obstacles on both sides - analyze front section and alternate turns
    size_t frontObstacles = 0;
    size_t frontCenterCols = GRID_WIDTH / 4; // Center quarter columns
    size_t startCol = (GRID_WIDTH - frontCenterCols) / 2;
    
    for (size_t y = GRID_HEIGHT / 2; y < GRID_HEIGHT; y++) { // Front half
        for (size_t x = startCol; x < startCol + frontCenterCols; x++) {
            if (obstacleGrid[y][x]) {
                frontObstacles++;
            }
        }
    }
    
    // If front is blocked with equal side obstacles, alternate turn preference
    if (frontObstacles > 0) {
        static bool preferRight = false;
        preferRight = !preferRight; // Alternate between left and right
        return preferRight ? AvoidanceCommand::TURN_RIGHT : AvoidanceCommand::TURN_LEFT;
    }
    
    return AvoidanceCommand::FORWARD;
}

cv::Mat ObstacleAvoidance::getGridVisualization() const {
    // Occupancy in white on black background
    cv::Mat grid(static_cast<int>(GRID_HEIGHT), static_cast<int>(GRID_WIDTH), CV_8UC1);
    for (size_t y = 0; y < GRID_HEIGHT; ++y) {
        const uint8_t* rowSrc = lastGrid_[y].data();
        uint8_t* rowDst = grid.ptr<uint8_t>(static_cast<int>(y));
        std::memcpy(rowDst, rowSrc, GRID_WIDTH);
    }
    return grid;
}
