#include "picoflexx.hpp"
#include <cmath>

// Set to 1 for debug output, 0 for release
#define DEBUG_MODE 0

picoflexx::picoflexx() {
    // Initialize cached frame
    cachedFrame_.hasData = false;
    cachedFrame_.width = 0;
    cachedFrame_.height = 0;

    if(!setUpCamera()) {
        std::cerr << "Can't setup camera - no point cloud data will be available" << std::endl;
    }
}

picoflexx::~picoflexx(){}

bool picoflexx::setUpCamera()
{
    // the camera manager will query for a connected camera
    royale::CameraManager manager;

    // checks if there are cameras connected
    if (manager.getConnectedCameraList().empty())
    {
        std::cout << "Could not find at least one connected Camera." << std::endl;
        return false;
    }

    // gets the first camera found
    m_cameraDevice = manager.createCamera (manager.getConnectedCameraList().first());

    // IMPORTANT: call the initialize method before working with the camera device
    auto ret = m_cameraDevice->initialize();
    if (ret != royale::CameraStatus::SUCCESS)
    {
        std::cout << "Camera device did not initialize: " << static_cast<int> (ret) << std::endl;
        return false;
    }

    // register data listener - will call onNewData
    ret = m_cameraDevice->registerDataListener (static_cast<IDepthDataListener *> (this));
    if (ret != royale::CameraStatus::SUCCESS)
    {
        std::cout << "Could not register data listener: " << static_cast<int> (ret) << std::endl;
        return false;
    }

     royale::Vector<royale::String> useCases;
     auto usecase_ = m_cameraDevice->getUseCases(useCases);

     desired_usecase_index = 0; // Initialize to first use case
     for(size_t i=0; i < useCases.size(); i++){
        if(useCases[i] == "Mode_9_30fps") {
            desired_usecase_index = i;
            break;
        }
     }

     // Safety check for bounds
     if (useCases.empty()) {
        std::cout << "No use cases available" << std::endl;
        return false;
     }

    ret = m_cameraDevice->setUseCase(useCases[desired_usecase_index]);
    if (ret != royale::CameraStatus::SUCCESS)
    {
        std::cout << "Cannot set usecase: " << static_cast<int> (ret) << std::endl;
        return false;
    }
    else{
        std::cout << "Use Case:  " << useCases[desired_usecase_index] << std::endl;
    }
    
     // Retrieve the IDs of the different streams
    royale::Vector<royale::StreamId> streamIds;
    if (m_cameraDevice->getStreams (streamIds) != royale::CameraStatus::SUCCESS)
    {
        std::cerr << "Error retrieving streams" << std::endl;
        return 1;
    }
 
    std::cout << "Stream IDs : ";
    for (auto curStream : streamIds)
    {
        std::cout << curStream << ", ";
    }
    std::cout << std::endl;
    
    // if (m_cameraDevice->setExposureTime (50, streamIds[0]) != royale::CameraStatus::SUCCESS)
    // {
    //     std::cerr << "Cannot set exposure time for stream" << streamIds[0] << std::endl;
    // }
    // else
    // {
    //     std::cout << "Changed exposure time for stream " << streamIds[0] << " to 200 microseconds ..." << std::endl;
    // }

        // start capturing
    ret = m_cameraDevice->startCapture();
    if (ret != royale::CameraStatus::SUCCESS)
    {
        std::cout << "Cannot start capturing: " << static_cast<int> (ret) << std::endl;
        return false;
    }
    std::cout << "Start capturing... " << std::endl;


    return true;
}

void picoflexx::onNewData(const royale::DepthData *data) {
    // Copy all data inside callback (data pointer only valid here!)
    FrameData frame;
    frame.width = data->width;
    frame.height = data->height;
    frame.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(data->timeStamp);

    auto numPoints = data->getNumPoints();

    frame.depthImage.resize(numPoints);
    frame.grayImage.resize(numPoints);
    frame.pointCloud.reserve(numPoints);

    // Iterate over all points (like ROS onNewData)
    for (size_t i = 0; i < numPoints; ++i) {
        auto confidence = data->getLegacyPoint(i).depthConfidence;
        float z = data->getLegacyPoint(i).z;

        if (confidence > 0) {
            point3d point;
            point.x = data->getLegacyPoint(i).x;
            point.y = data->getLegacyPoint(i).y;
            point.z = z;
            frame.pointCloud.push_back(point);
            frame.depthImage[i] = z;
        } else {
            frame.depthImage[i] = 0.0f;
        }

        frame.grayImage[i] = data->getLegacyPoint(i).grayValue;
    }

    frame.hasData = true;

    // Store in cached frame (thread-safe)
    {
        std::lock_guard<std::mutex> lock(frameMutex_);
        cachedFrame_ = std::move(frame);
    }
}

FrameData picoflexx::getFrameData() {
    std::lock_guard<std::mutex> lock(frameMutex_);
    return cachedFrame_;
}

