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
// Toggle OpenCV obstacle grid window
#define SHOW_GRID 0

// Rotate depth and gray images 90° CCW to match point cloud transform
static void rotateFrameImages(FrameData& frame) {
    if (frame.width == 0 || frame.height == 0 ||
        frame.depthImage.empty() || frame.grayImage.empty()) {
        return;
    }

    const uint16_t origWidth = frame.width;
    const uint16_t origHeight = frame.height;
    const uint32_t newWidth = origHeight;
    const uint32_t newHeight = origWidth;

    std::vector<float> rotatedDepth(newWidth * newHeight, 0.0f);
    std::vector<uint16_t> rotatedGray(newWidth * newHeight, 0);

    for (uint32_t y = 0; y < origHeight; ++y) {
        for (uint32_t x = 0; x < origWidth; ++x) {
            uint32_t srcIdx = y * origWidth + x;
            uint32_t dstX = y;
            uint32_t dstY = origWidth - 1 - x;
            uint32_t dstIdx = dstY * newWidth + dstX;
            rotatedDepth[dstIdx] = frame.depthImage[srcIdx];
            rotatedGray[dstIdx] = frame.grayImage[srcIdx];
        }
    }

    frame.width = static_cast<uint16_t>(newWidth);
    frame.height = static_cast<uint16_t>(newHeight);
    frame.depthImage.swap(rotatedDepth);
    frame.grayImage.swap(rotatedGray);
}

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
            // rotateFrameImages(frame);
            // Rotate -90° around Z-axis to align camera frame
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

// FreeRTOS Task: UDP Depth Image Sending
// Sends fixed 224x172 uint8 depth image (38,528 bytes per frame)
void vUDPTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // ~30 FPS

    const int UDP_WIDTH = 224;
    const int UDP_HEIGHT = 172;
    const float MAX_DEPTH = 4.0f;  // Max depth in meters for normalization

#if DEBUG_MODE
    std::cout << "UDP Task started (sending " << UDP_WIDTH << "x" << UDP_HEIGHT << " depth)" << std::endl;
#endif

    while (1) {
        uint16_t width = 0, height = 0;
        std::vector<float> depthImage;

        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (sharedFrameData.width > 0 && !sharedFrameData.depthImage.empty()) {
                width = sharedFrameData.width;
                height = sharedFrameData.height;
                depthImage = sharedFrameData.depthImage;
            }
            xSemaphoreGive(xDataMutex);
        }

        if (width > 0 && height > 0 && !depthImage.empty()) {
            // Create OpenCV Mat from depth data
            cv::Mat depthMat(height, width, CV_32FC1, depthImage.data());

            // Resize to fixed dimensions
            cv::Mat resizedDepth;
            cv::resize(depthMat, resizedDepth, cv::Size(UDP_WIDTH, UDP_HEIGHT), 0, 0, cv::INTER_LINEAR);

            // Convert to uint8 (0-255) for viewing
            cv::Mat depthU8;
            resizedDepth.convertTo(depthU8, CV_8UC1, 255.0 / MAX_DEPTH);

            // Send raw depth data via UDP
            if (udp_socket >= 0) {
                ssize_t sent = sendto(udp_socket, depthU8.data, UDP_WIDTH * UDP_HEIGHT, 0,
                                     (struct sockaddr*)&viewer_addr, sizeof(viewer_addr));
                if (sent < 0) {
                    std::cerr << "UDP send failed: " << strerror(errno) << std::endl;
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// FreeRTOS Task: Depth Image Display (like ROS depth_image topic)
void vDepthImageTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33); // 20 FPS for display

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

            // Create gray image Mat (like ROS MONO16)
            cv::Mat grayMat(height, width, CV_16UC1, grayImage.data());
            cv::Mat grayNorm;
            grayMat.convertTo(grayNorm, CV_8UC1, 255.0 / 1000.0);

            // Display images (startWindowThread handles GUI updates)
            // cv::imshow("Depth Image", depthColor);
            // cv::imshow("Gray Image", grayNorm);
            // cv::waitKey(1);
        }

        if (SHOW_GRID) {
            cv::Mat grid = obstacle_avoidance.getGridVisualization();
            if (!grid.empty()) {
                cv::Mat gridScaled;
                cv::resize(grid, gridScaled, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
                cv::imshow("Obstacle Grid", gridScaled);
                cv::waitKey(1);
            }
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
