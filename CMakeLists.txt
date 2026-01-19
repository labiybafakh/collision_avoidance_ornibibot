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
find_package(GLUT REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common io filters features segmentation surface sample_consensus)
find_package(OpenCV REQUIRED)

# FreeRTOS Configuration
set(FREERTOS_KERNEL_PATH ${CMAKE_SOURCE_DIR}/third_party/FreeRTOS-Kernel)
set(FREERTOS_PORT_PATH ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/Posix)

# FreeRTOS Source Files
set(FREERTOS_SOURCES
    ${FREERTOS_KERNEL_PATH}/tasks.c
    ${FREERTOS_KERNEL_PATH}/list.c
    ${FREERTOS_KERNEL_PATH}/queue.c
    ${FREERTOS_KERNEL_PATH}/timers.c
    ${FREERTOS_KERNEL_PATH}/event_groups.c
    ${FREERTOS_KERNEL_PATH}/stream_buffer.c
    ${FREERTOS_KERNEL_PATH}/portable/MemMang/heap_3.c
    ${FREERTOS_PORT_PATH}/port.c
    ${FREERTOS_PORT_PATH}/utils/wait_for_event.c
)

# Create FreeRTOS library
add_library(freertos STATIC ${FREERTOS_SOURCES})


# Add FreeRTOS processing executable
add_executable(picoflexx_processor
    src/main_freertos.cpp
    src/picoflexx.cpp
    src/pointcloud_utils.cpp
    src/obstacle_avoidance.cpp
    src/freertos_hooks.c
)

# Add simple viewer executable (no FreeRTOS)
add_executable(pointcloud_viewer
    src/viewer.cpp
)

# FreeRTOS Include directories
target_include_directories(freertos PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_PORT_PATH}
)

target_compile_definitions(freertos PRIVATE
    projCOVERAGE_TEST=0
)

# Include directories for FreeRTOS processor
target_include_directories(picoflexx_processor PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${royale_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${FREERTOS_KERNEL_PATH}/include
    ${FREERTOS_PORT_PATH}
)

# Include directories for viewer
target_include_directories(pointcloud_viewer PRIVATE
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
)

# Link libraries for FreeRTOS processor
target_link_libraries(picoflexx_processor PRIVATE
    ${ROYALE_LIB}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    freertos
    pthread
)

# Link libraries for viewer
target_link_libraries(pointcloud_viewer PRIVATE
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    pthread
)