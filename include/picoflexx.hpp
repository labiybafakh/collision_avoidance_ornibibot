#ifndef PICOFLEXX
#define PICOFLEXX

#include <royale.hpp>
#include <GL/freeglut.h>
#include <atomic>
#include <string>
#include <chrono>
#include <vector>
#include <string.h>
#include <math.h>

struct point3d{
    float x;
    float y;
    float z;
};

class picoflexx: public royale::IDepthDataListener{
    public:
        void onNewData (const royale::DepthData *data) override;
        std::vector<point3d> getPointCloud();
        picoflexx();
        ~picoflexx();
    private:
        bool setUpCamera();
        std::unique_ptr<royale::ICameraDevice> m_cameraDevice;
        int iterX, iterY, iterZ;
        int pointcloud_size_;
        const royale::DepthData *depth_data_;
        size_t desired_usecase_index;

};

#endif