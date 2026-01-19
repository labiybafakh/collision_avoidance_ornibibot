#ifndef PICOFLEXX
#define PICOFLEXX

#include <royale.hpp>
#include <GL/freeglut.h>
#include <atomic>
#include <string>
#include <chrono>
#include <vector>
#include <mutex>
#include <string.h>
#include <math.h>

struct point3d{
    float x;
    float y;
    float z;
};

struct FrameData {
    uint16_t width;
    uint16_t height;
    std::vector<point3d> pointCloud;
    std::vector<float> depthImage;      // 32-bit float depth (Z values)
    std::vector<uint16_t> grayImage;    // 16-bit grayscale
    std::chrono::microseconds timestamp;
    bool hasData;
};

class picoflexx: public royale::IDepthDataListener{
    public:
        void onNewData (const royale::DepthData *data) override;
        FrameData getFrameData();
        picoflexx();
        ~picoflexx();
    private:
        bool setUpCamera();
        std::unique_ptr<royale::ICameraDevice> m_cameraDevice;
        size_t desired_usecase_index;

        // Cached frame data (copied in onNewData callback)
        FrameData cachedFrame_;
        std::mutex frameMutex_;
};

#endif