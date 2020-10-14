# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS} eigen)

add_executable(PlotCameraFrames main.cpp)
target_link_libraries(PlotCameraFrames ${Pangolin_LIBRARIES})
