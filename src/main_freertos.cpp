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
#include <chrono>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Set to 1 for debug output, 0 for release
#define DEBUG_MODE 0

picoflexx picoflexx_;
PointCloudUtils pc_utils;
ObstacleAvoidance obstacle_avoidance;

// FreeRTOS handles
TaskHandle_t xPointCloudTaskHandle = NULL;
TaskHandle_t xUDPTaskHandle = NULL;
TaskHandle_t xCollisionAvoidanceTaskHandle = NULL;
TaskHandle_t xDepthImageTaskHandle = NULL;
SemaphoreHandle_t xDataMutex = NULL;

// Shared frame data (like ROS)
FrameData sharedFrameData;

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
    
#if DEBUG_MODE
    std::cout << "UDP socket initialized, will send to 127.0.0.1:8080" << std::endl;
#endif
    return true;
}

// FreeRTOS Task: Frame Data Acquisition (like ROS onNewData)
void vPointCloudTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS

#if DEBUG_MODE
    std::cout << "Point Cloud Task started" << std::endl;
#endif

    while (1) {
        // Get all frame data at once (like ROS)
        FrameData frame = picoflexx_.getFrameData();

        if (frame.hasData) {
            // Rotate -90° around Z-axis to align camera frame
            PointCloudUtils::rotateZ90(frame.pointCloud);

            if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
                sharedFrameData = frame;
                xSemaphoreGive(xDataMutex);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// FreeRTOS Task: Collision Avoidance Processing
void vCollisionAvoidanceTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS

    AvoidanceCommand lastCommand = AvoidanceCommand::FORWARD;

    while (1) {
        std::vector<point3d> cloud;
        std::chrono::microseconds timestamp{0};

        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            cloud = sharedFrameData.pointCloud;
            timestamp = sharedFrameData.timestamp;
            xSemaphoreGive(xDataMutex);
        }

        if (!cloud.empty()) {
            // Filter raw point cloud to navigation envelope
            std::vector<point3d> filtered_cloud;
            const auto& params = obstacle_avoidance.getParams();

            for (const auto& point : cloud) {
                if (point.z >= params.minRange && point.z < params.maxRange &&
                    abs(point.x) <= params.maxXDistance && abs(point.y) <= params.maxYDistance) {
                    filtered_cloud.push_back(point);
                }
            }

            // Process obstacle avoidance
            AvoidanceCommand avoidance_cmd = obstacle_avoidance.processPointCloud(filtered_cloud);

            // Output navigation command with camera timestamp (like ROS)
            const char* cmdStr;
            switch(avoidance_cmd) {
                case AvoidanceCommand::FORWARD: cmdStr = "FORWARD"; break;
                case AvoidanceCommand::TURN_LEFT: cmdStr = "TURN_LEFT"; break;
                case AvoidanceCommand::TURN_RIGHT: cmdStr = "TURN_RIGHT"; break;
                case AvoidanceCommand::STOP: cmdStr = "STOP"; break;
                default: cmdStr = "UNKNOWN"; break;
            }
            uint64_t ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp).count();
            uint64_t sec = ts_ns / 1000000000;
            uint64_t nsec = ts_ns % 1000000000;
            std::cout << "[" << sec << "." << std::setfill('0') << std::setw(9) << nsec << "] " << cmdStr << "\n";
            lastCommand = avoidance_cmd;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// FreeRTOS Task: UDP Point Cloud Sending
void vUDPTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS

#if DEBUG_MODE
    std::cout << "UDP Task started" << std::endl;
#endif

    uint32_t frameCount = 0;

    while (1) {
        std::vector<point3d> cloud;

        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            cloud = sharedFrameData.pointCloud;
            xSemaphoreGive(xDataMutex);
        }

        if (!cloud.empty()) {
            frameCount++;

            // Filter point cloud for UDP transmission
            std::vector<point3d> filtered_cloud;
            const auto& params = obstacle_avoidance.getParams();

            for (const auto& point : cloud) {
                if (point.z >= params.minRange && point.z < params.maxRange &&
                    abs(point.x) <= params.maxXDistance && abs(point.y) <= params.maxYDistance) {
                    filtered_cloud.push_back(point);
                }
            }

            // Downsample for UDP transmission
            auto pcl_cloud = pc_utils.convertToPCL(filtered_cloud);
            auto downsampled_pcl_cloud = pc_utils.downsamplePCL(pcl_cloud, 0.015);

#if DEBUG_MODE
            if (frameCount % 30 == 0) {
                std::cout << "[UDP] Frame " << frameCount << ": "
                          << cloud.size() << " -> " << filtered_cloud.size()
                          << " (filtered) -> " << downsampled_pcl_cloud.size()
                          << " points (final)" << std::endl;
            }
#endif

            // Send via UDP
            if (udp_socket >= 0 && !pcl_cloud.empty()) {
                std::vector<point3d> udp_points;
                for (const auto& pt : pcl_cloud.points) {
                    udp_points.push_back({pt.x, pt.y, pt.z});
                }

                const size_t MAX_UDP_SIZE = 8192;
                const size_t max_points = MAX_UDP_SIZE / sizeof(point3d);

                if (udp_points.size() > max_points) {
                    std::vector<point3d> subsampled_points;
                    subsampled_points.reserve(max_points);

                    double step = static_cast<double>(udp_points.size()) / max_points;
                    for (size_t i = 0; i < max_points && (static_cast<size_t>(i * step) < udp_points.size()); ++i) {
                        size_t idx = static_cast<size_t>(i * step);
                        subsampled_points.push_back(udp_points[idx]);
                    }
                    udp_points = std::move(subsampled_points);
                }

                size_t data_size = udp_points.size() * sizeof(point3d);
                ssize_t sent = sendto(udp_socket, udp_points.data(), data_size, 0,
                                     (struct sockaddr*)&viewer_addr, sizeof(viewer_addr));
                if (sent < 0) {
                    std::cerr << "UDP send failed: " << strerror(errno) << " (size: " << data_size << " bytes)" << std::endl;
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// FreeRTOS Task: Depth Image Display (like ROS depth_image topic)
void vDepthImageTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 FPS for display

    const float maxFilter = 4.0f;  // Max depth in meters

    while (1) {
        uint16_t width = 0, height = 0;
        std::vector<float> depthImage;
        std::vector<uint16_t> grayImage;

        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (sharedFrameData.width > 0 && !sharedFrameData.depthImage.empty()) {
                width = sharedFrameData.width;
                height = sharedFrameData.height;
                depthImage = sharedFrameData.depthImage;
                grayImage = sharedFrameData.grayImage;
            }
            xSemaphoreGive(xDataMutex);
        }

        if (width > 0 && height > 0 && !depthImage.empty()) {
            // Create OpenCV Mat from depth data (like ROS TYPE_32FC1)
            cv::Mat depthMat(height, width, CV_32FC1, depthImage.data());

            // Normalize depth to 0-255 for visualization
            cv::Mat depthNorm;
            depthMat.convertTo(depthNorm, CV_8UC1, 255.0 / maxFilter);

            // Apply colormap for better visualization (like RViz)
            cv::Mat depthColor;
            cv::applyColorMap(depthNorm, depthColor, cv::COLORMAP_JET);

            // Rotate -90 degrees (counterclockwise)
            cv::Mat depthRotated;
            cv::rotate(depthColor, depthRotated, cv::ROTATE_90_COUNTERCLOCKWISE);

            // Create gray image Mat (like ROS MONO16)
            cv::Mat grayMat(height, width, CV_16UC1, grayImage.data());
            cv::Mat grayNorm;
            grayMat.convertTo(grayNorm, CV_8UC1, 255.0 / 1000.0);

            // Rotate -90 degrees (counterclockwise)
            cv::Mat grayRotated;
            cv::rotate(grayNorm, grayRotated, cv::ROTATE_90_COUNTERCLOCKWISE);

            // Display images (startWindowThread handles GUI updates)
            cv::imshow("Depth Image", depthRotated);
            cv::imshow("Gray Image", grayRotated);
            cv::waitKey(1);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

int main(int argc, char** argv) {
#if DEBUG_MODE
    std::cout << "Starting FreeRTOS Point Cloud Processing Application..." << std::endl;
#endif
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
    sharedFrameData.hasData = false;
    sharedFrameData.width = 0;
    sharedFrameData.height = 0;

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

    result = xTaskCreate(vCollisionAvoidanceTask, "CollisionAvoidanceTask", 4096, NULL, 3, &xCollisionAvoidanceTaskHandle);
    if (result != pdPASS) {
        std::cerr << "Failed to create CollisionAvoidanceTask" << std::endl;
        return -1;
    }

    result = xTaskCreate(vDepthImageTask, "DepthImageTask", 8192, NULL, 1, &xDepthImageTaskHandle);
    if (result != pdPASS) {
        std::cerr << "Failed to create DepthImageTask" << std::endl;
        return -1;
    }

#if DEBUG_MODE
    std::cout << "FreeRTOS tasks created successfully. Starting scheduler..." << std::endl;
#endif

    // Start FreeRTOS scheduler
    vTaskStartScheduler();
    
    // Should never reach here
    std::cerr << "FreeRTOS scheduler returned unexpectedly" << std::endl;
    return -1;
}