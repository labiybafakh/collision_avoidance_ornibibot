#include "picoflexx.hpp"
#include <cmath>

picoflexx::picoflexx(){
    if(!setUpCamera()) {
        std::cerr << "Can't setup camera - no point cloud data will be available" << std::endl;
        depth_data_ = nullptr;
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

     for(size_t i=0; i < useCases.size(); i++){
        if(useCases[i] == "MODE_9_25FPS_450") desired_usecase_index = i;
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
    
    if (m_cameraDevice->setExposureTime (50, streamIds[0]) != royale::CameraStatus::SUCCESS)
    {
        std::cerr << "Cannot set exposure time for stream" << streamIds[0] << std::endl;
    }
    else
    {
        std::cout << "Changed exposure time for stream " << streamIds[0] << " to 200 microseconds ..." << std::endl;
    }

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

void picoflexx::onNewData (const royale::DepthData *data){

    depth_data_ = data;
}

std::vector<point3d> picoflexx::getPointCloud(){
    std::vector<point3d> point_cloud_data;

    // If no camera data available, return empty cloud
    if (!depth_data_) {
        return point_cloud_data; // Return empty vector
    }

    auto nums_points = depth_data_->getNumPoints();

    // Debug: Sample some points to check x,y values
    static int debug_counter = 0;
    bool should_debug = (debug_counter++ % 30 == 0); // Debug every 30th frame
    
    for (size_t i = 0; i < nums_points; ++i) {
        if (depth_data_->getLegacyPoint(i).depthConfidence > 1) {
            point3d point;
            point.x = depth_data_->getLegacyPoint(i).x;
            point.y = depth_data_->getLegacyPoint(i).y;
            point.z = depth_data_->getLegacyPoint(i).z;
            point_cloud_data.emplace_back(point);
            
            // Debug first few points occasionally
            if (should_debug && point_cloud_data.size() <= 5) {
                std::cout << "Point " << point_cloud_data.size() << ": x=" << point.x 
                         << ", y=" << point.y << ", z=" << point.z << std::endl;
            }
        }
    }

    return point_cloud_data;
}

