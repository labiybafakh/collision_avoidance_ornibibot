#include <iostream>
#include <cmath>
#include "picoflexx.hpp"
#include "pointcloud_utils.hpp"
#include "obstacle_avoidance.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

picoflexx picoflexx_;
PointCloudUtils pc_utils;
ObstacleAvoidance obstacle_avoidance;

// FreeRTOS handles
TaskHandle_t xPointCloudTaskHandle = NULL;
TaskHandle_t xUDPTaskHandle = NULL;
SemaphoreHandle_t xDataMutex = NULL;

// Shared data structure
struct PointCloudData {
    std::vector<point3d> cloud;
    bool dataReady;
};

PointCloudData sharedPointCloudData;

// UDP socket
int udp_socket = -1;
struct sockaddr_in viewer_addr;

// Initialize UDP socket
bool initUDP() {
    udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
        std::cerr << "Failed to create UDP socket" << std::endl;
        return false;
    }
    
    memset(&viewer_addr, 0, sizeof(viewer_addr));
    viewer_addr.sin_family = AF_INET;
    viewer_addr.sin_port = htons(8080);
    viewer_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    
    std::cout << "UDP socket initialized, will send to 127.0.0.1:8080" << std::endl;
    return true;
}

// FreeRTOS Task: Point Cloud Data Acquisition
void vPointCloudTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS
    
    std::cout << "Point Cloud Task started" << std::endl;
    
    while (1) {
        auto cloud = picoflexx_.getPointCloud();
        
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            sharedPointCloudData.cloud = cloud;
            sharedPointCloudData.dataReady = true;
            xSemaphoreGive(xDataMutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// FreeRTOS Task: Point Cloud Processing and UDP Sending
void vUDPTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS
    
    std::cout << "UDP Task started" << std::endl;
    
    uint32_t frameCount = 0;
    
    while (1) {
        std::vector<point3d> cloud;
        bool hasData = false;
        
        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (sharedPointCloudData.dataReady) {
                cloud = sharedPointCloudData.cloud;
                hasData = true;
                sharedPointCloudData.dataReady = false;
            }
            xSemaphoreGive(xDataMutex);
        }
        
        if (hasData) {
            frameCount++;
            
            if (cloud.empty()) {
                // Print warning for empty clouds every 60 frames (~2 seconds)
                if (frameCount % 60 == 0) {
                    std::cout << "Warning: No point cloud data available. Check camera connection." << std::endl;
                }
            } else {
                // First, filter raw point cloud to navigation envelope to preserve obstacle data
                std::vector<point3d> filtered_cloud;
                const auto& params = obstacle_avoidance.getParams();
                
                for (const auto& point : cloud) {
                    // Keep points within navigation corridor (same as obstacle avoidance envelope)
                    if (point.z >= params.minRange && point.z < params.maxRange &&
                        abs(point.x) <= params.maxXDistance && abs(point.y) <= params.maxYDistance) {
                        filtered_cloud.push_back(point);
                    }
                }
                
                // Process obstacle avoidance with filtered (but not downsampled) data
                AvoidanceCommand avoidance_cmd = obstacle_avoidance.processPointCloud(filtered_cloud);
                const auto& stats = obstacle_avoidance.getStats();
                
                // Print avoidance information every 30 frames
                if (frameCount % 30 == 0) {
                    const char* cmdStr;
                    switch(avoidance_cmd) {
                        case AvoidanceCommand::FORWARD: cmdStr = "FORWARD"; break;
                        case AvoidanceCommand::TURN_LEFT: cmdStr = "TURN_LEFT"; break;
                        case AvoidanceCommand::TURN_RIGHT: cmdStr = "TURN_RIGHT"; break;
                        case AvoidanceCommand::STOP: cmdStr = "STOP"; break;
                        default: cmdStr = "UNKNOWN"; break;
                    }
                    std::cout << "  -> Obstacle Avoidance: " << cmdStr
                              << " (Obstacles: " << stats.obstaclePoints << "/" << stats.totalPoints 
                              << ", Min Distance: " << stats.minDistance << "m)" << std::endl;
                }
                
                // Output navigation command (could be sent to robot control system)
                if (avoidance_cmd != AvoidanceCommand::FORWARD) {
                    // This could interface with your robot's control system
                    // For now, just demonstrate the functionality
                }
                
                // Now downsample the filtered data for UDP transmission (preserves navigation-relevant data)
                auto pcl_cloud = pc_utils.convertToPCL(filtered_cloud);
                auto downsampled_pcl_cloud = pc_utils.downsamplePCL(pcl_cloud, 0.015); // Minimal downsampling for maximum point density
                
                // Print stats every 30 frames (~1 second)
                if (frameCount % 30 == 0) {
                    std::cout << "Frame " << frameCount << ": " 
                              << cloud.size() << " -> " << filtered_cloud.size() 
                              << " (filtered) -> " << downsampled_pcl_cloud.size() 
                              << " points (final)" << std::endl;
                }
                
                // Send via UDP
                if (udp_socket >= 0 && !pcl_cloud.empty()) {
                    // Convert PCL points to our Point3D structure for UDP
                    std::vector<point3d> udp_points;
                    for (const auto& pt : pcl_cloud.points) {
                        udp_points.push_back({pt.x, pt.y, pt.z});
                    }
                    
                    // Optimize UDP packet size to send maximum points
                    const size_t MAX_UDP_SIZE = 8192; // 8KB - very conservative UDP size for all systems
                    const size_t max_points = MAX_UDP_SIZE / sizeof(point3d);
                    
                    if (udp_points.size() > max_points) {
                        // Smart subsampling: preserve spatial distribution
                        std::vector<point3d> subsampled_points;
                        subsampled_points.reserve(max_points);
                        
                        // Use uniform distribution across the point cloud
                        double step = static_cast<double>(udp_points.size()) / max_points;
                        for (size_t i = 0; i < max_points && (static_cast<size_t>(i * step) < udp_points.size()); ++i) {
                            size_t idx = static_cast<size_t>(i * step);
                            subsampled_points.push_back(udp_points[idx]);
                        }
                        udp_points = std::move(subsampled_points);
                    }
                    
                    // Always send exactly max_points when possible
                    if (udp_points.size() < max_points && !pcl_cloud.empty()) {
                        // If we have fewer points than max, we could use less aggressive downsampling
                        // But keep current points for this frame
                    }
                    
                    size_t data_size = udp_points.size() * sizeof(point3d);
                    ssize_t sent = sendto(udp_socket, udp_points.data(), data_size, 0,
                                         (struct sockaddr*)&viewer_addr, sizeof(viewer_addr));
                    if (sent < 0) {
                        std::cerr << "UDP send failed: " << strerror(errno) << " (size: " << data_size << " bytes)" << std::endl;
                    } else if (frameCount % 30 == 0) {
                        std::cout << "  -> Sent " << udp_points.size() << "/" << max_points << " points (" << data_size << " bytes) via UDP" << std::endl;
                    }
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

int main(int argc, char** argv) {
    std::cout << "Starting FreeRTOS Point Cloud Processing Application..." << std::endl;
    
    // Initialize UDP
    if (!initUDP()) {
        return -1;
    }
    
    // Initialize FreeRTOS primitives
    xDataMutex = xSemaphoreCreateMutex();
    if (xDataMutex == NULL) {
        std::cerr << "Failed to create mutex" << std::endl;
        return -1;
    }

    // Initialize shared data
    sharedPointCloudData.dataReady = false;

    // Create FreeRTOS tasks
    BaseType_t result;
    
    result = xTaskCreate(vPointCloudTask, "PointCloudTask", 4096, NULL, 3, &xPointCloudTaskHandle);
    if (result != pdPASS) {
        std::cerr << "Failed to create PointCloudTask" << std::endl;
        return -1;
    }
    
    result = xTaskCreate(vUDPTask, "UDPTask", 8192, NULL, 2, &xUDPTaskHandle);
    if (result != pdPASS) {
        std::cerr << "Failed to create UDPTask" << std::endl;
        return -1;
    }

    std::cout << "FreeRTOS tasks created successfully. Starting scheduler..." << std::endl;
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    std::cerr << "FreeRTOS scheduler returned unexpectedly" << std::endl;
    return -1;
}