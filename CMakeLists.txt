cmake_minimum_required(VERSION 3.10)
project(picoflexx_pointcloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# --- Royale (libroyale) ---
set(royale_DIR ${CMAKE_SOURCE_DIR}/third_party/libroyale)
set(royale_BIN_DIR ${royale_DIR}/bin)

# Find platform-specific library in bin/
if(APPLE)
  set(ROYALE_LIB ${royale_BIN_DIR}/libroyale.dylib)
elseif(UNIX)
  set(ROYALE_LIB ${royale_BIN_DIR}/libroyale.so)
endif()

# Find packages
#find_package(royale REQUIRED)
find_package(OpenGL REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED)

# Add simple viewer executable (no FreeRTOS)
add_executable(pointcloud_viewer
    src/viewer.cpp
)

# Add Linux threading executable (no FreeRTOS)
add_executable(collision_avoidance_ornibibot
    src/main.cpp
    src/picoflexx.cpp
    src/pointcloud_utils.cpp
    src/obstacle_avoidance.cpp
)

# Include directories for viewer
target_include_directories(pointcloud_viewer PRIVATE
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)



# Include directories for Linux threading executable
target_include_directories(collision_avoidance_ornibibot PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${royale_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Link libraries for viewer
target_link_libraries(pointcloud_viewer PRIVATE
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${OpenCV_LIBS}
    pthread
)

# Link libraries for Linux threading executable
target_link_libraries(collision_avoidance_ornibibot PRIVATE
    ${ROYALE_LIB}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    pthread
)