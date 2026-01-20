#include <iostream>
#include <cmath>
#include "picoflexx.hpp"
#include "obstacle_avoidance.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <chrono>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <sys/select.h>
#include <fcntl.h>
#include <termios.h>

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

// Global objects
picoflexx picoflexx_;
ObstacleAvoidance obstacle_avoidance;

// Shared data protection
std::mutex dataMutex;
std::atomic<bool> shouldExit{false};

// Shared frame data
FrameData sharedFrameData;

// Shared display data for main thread
cv::Mat sharedDepthColor, sharedGrayNorm, sharedGridScaled;
std::atomic<bool> displayDataReady{false};

// Shared serial command data
std::atomic<AvoidanceCommand> currentCommand{AvoidanceCommand::FORWARD};

// UDP socket
int udp_socket = -1;
struct sockaddr_in viewer_addr;

// Serial port
int serial_fd = -1;
const char* SERIAL_PORT = "/dev/ttyUSB0";

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

// Initialize serial port
bool initSerial() {
    serial_fd = open(SERIAL_PORT, O_WRONLY | O_NOCTTY);
    if (serial_fd < 0) {
        std::cerr << "Failed to open serial port " << SERIAL_PORT << std::endl;
        return false;
    }
    
    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting serial port attributes" << std::endl;
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    
    // Configure serial port: 9600 baud, 8N1
    cfsetospeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial port attributes" << std::endl;
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    
#if DEBUG_MODE
    std::cout << "Serial port " << SERIAL_PORT << " initialized" << std::endl;
#endif
    return true;
}

// Thread: Serial Communication
void serialTask() {
#if DEBUG_MODE
    std::cout << "Serial Task started" << std::endl;
#endif
    
    while (!shouldExit.load()) {
        AvoidanceCommand cmd = currentCommand.load();
        
        if (serial_fd >= 0) {
            uint8_t roll_value = 127; // Middle value (0 degrees)
            
            switch(cmd) {
                case AvoidanceCommand::TURN_LEFT:
                    roll_value = 127 - 25; // Target angle -25 degrees
                    break;
                case AvoidanceCommand::TURN_RIGHT:
                    roll_value = 127 + 25; // Target angle +25 degrees
                    break;
                case AvoidanceCommand::FORWARD:
                case AvoidanceCommand::STOP:
                default:
                    roll_value = 127; // 0 degrees (forward)
                    break;
            }
            
            ssize_t written = write(serial_fd, &roll_value, 1);
            if (written != 1) {
                std::cerr << "Serial write failed: " << strerror(errno) << std::endl;
            }
#if DEBUG_MODE
            else {
                std::cout << "Sent roll command: " << static_cast<int>(roll_value) << std::endl;
            }
#endif
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 Hz send rate
    }
}

// Thread: Frame Data Acquisition (like ROS onNewData)
void pointCloudTask() {
#if DEBUG_MODE
    std::cout << "Point Cloud Task started" << std::endl;
#endif

    while (!shouldExit.load()) {
        // Get all frame data at once (like ROS)
        FrameData frame = picoflexx_.getFrameData();

        if (frame.hasData) {
            // rotateFrameImages(frame);
            // Rotate -90° around Z-axis to align camera frame
            std::lock_guard<std::mutex> lock(dataMutex);
            sharedFrameData = frame;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
}

// Thread: Collision Avoidance Processing
void collisionAvoidanceTask() {
    AvoidanceCommand lastCommand = AvoidanceCommand::FORWARD;

    while (!shouldExit.load()) {
        std::vector<point3d> cloud;
        std::chrono::microseconds timestamp{0};

        {
            std::lock_guard<std::mutex> lock(dataMutex);
            cloud = sharedFrameData.pointCloud;
            timestamp = sharedFrameData.timestamp;
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
            
            // Update shared command for serial thread
            currentCommand.store(avoidance_cmd);
            
            lastCommand = avoidance_cmd;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
}

// Thread: UDP Depth Image Sending
void udpTask() {
    const int UDP_WIDTH = 224;
    const int UDP_HEIGHT = 172;
    const float MAX_DEPTH = 4.0f;  // Max depth in meters for normalization

#if DEBUG_MODE
    std::cout << "UDP Task started (sending " << UDP_WIDTH << "x" << UDP_HEIGHT << " depth)" << std::endl;
#endif

    while (!shouldExit.load()) {
        uint16_t width = 0, height = 0;
        std::vector<float> depthImage;

        {
            std::lock_guard<std::mutex> lock(dataMutex);
            if (sharedFrameData.width > 0 && !sharedFrameData.depthImage.empty()) {
                width = sharedFrameData.width;
                height = sharedFrameData.height;
                depthImage = sharedFrameData.depthImage;
            }
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

        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
}

// Thread: Prepare Image Display Data (like ROS depth_image topic)
void depthImageTask() {
    const float maxFilter = 4.0f;  // Max depth in meters

    while (!shouldExit.load()) {
        uint16_t width = 0, height = 0;
        std::vector<float> depthImage;
        std::vector<uint16_t> grayImage;

        {
            std::lock_guard<std::mutex> lock(dataMutex);
            if (sharedFrameData.width > 0 && !sharedFrameData.depthImage.empty()) {
                width = sharedFrameData.width;
                height = sharedFrameData.height;
                depthImage = sharedFrameData.depthImage;
                grayImage = sharedFrameData.grayImage;
            }
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

            // Prepare display data for main thread
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                sharedDepthColor = depthColor.clone();
                sharedGrayNorm = grayNorm.clone();
                
                if (SHOW_GRID) {
                    cv::Mat grid = obstacle_avoidance.getGridVisualization();
                    if (!grid.empty()) {
                        cv::resize(grid, sharedGridScaled, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
                    }
                }
                
                displayDataReady.store(true);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 FPS for display
    }
}

int main(int argc, char** argv) {
#if DEBUG_MODE
    std::cout << "Starting Linux Threaded Point Cloud Processing Application..." << std::endl;
#endif
    
    // Initialize UDP
    if (!initUDP()) {
        return -1;
    }
    
    Initialize Serial (required for operation)
    if (!initSerial()) {
        std::cerr << "Serial initialization failed. Cannot continue without serial communication." << std::endl;
        return -1;
    }

    // Initialize shared data
    sharedFrameData.hasData = false;
    sharedFrameData.width = 0;
    sharedFrameData.height = 0;

    // Create threads using std::thread
    std::thread pointCloudThread(pointCloudTask);
    // std::thread udpThread(udpTask);
    std::thread collisionThread(collisionAvoidanceTask);
    std::thread depthThread(depthImageTask);
    std::thread serialThread(serialTask);

#if DEBUG_MODE
    std::cout << "All threads created successfully. Running..." << std::endl;
#endif

    std::cout << "Press Enter to exit..." << std::endl;
    
    // Main thread handles OpenCV display (required for macOS)
    while (!shouldExit.load()) {
        if (displayDataReady.load()) {
            cv::Mat depthToShow, grayToShow, gridToShow;
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                if (!sharedDepthColor.empty()) depthToShow = sharedDepthColor.clone();
                if (!sharedGrayNorm.empty()) grayToShow = sharedGrayNorm.clone();
                if (SHOW_GRID && !sharedGridScaled.empty()) gridToShow = sharedGridScaled.clone();
                displayDataReady.store(false);
            }
            
            if (!depthToShow.empty()) {
                cv::imshow("Depth Image", depthToShow);
            }
            if (!grayToShow.empty()) {
                cv::imshow("Gray Image", grayToShow);
            }
            if (!gridToShow.empty()) {
                cv::imshow("Obstacle Grid", gridToShow);
            }
        }
        
        // Check for key press or if stdin has input
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') break;  // ESC or 'q' to exit
        
        // Non-blocking check for Enter key
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 1000;  // 1ms timeout
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0) {
            if (FD_ISSET(STDIN_FILENO, &readfds)) {
                break;  // Enter was pressed
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS display
    }
    
    // Signal threads to exit
    shouldExit.store(true);

    // Wait for threads to complete
    pointCloudThread.join();
    // udpThread.join();
    collisionThread.join();
    depthThread.join();
    // serialThread.join();

    // Cleanup
    if (udp_socket >= 0) {
        close(udp_socket);
    }
    if (serial_fd >= 0) {
        close(serial_fd);
    }

    std::cout << "Application terminated cleanly." << std::endl;
    return 0;
}
