cmake_minimum_required(VERSION 3.9)

project(ros_tegra_utilities)


################################################################################
# Include CMake dependencies
################################################################################

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

################################################################################
# Compiler specific configuration
################################################################################

set(CMAKE_CXX_STANDARD 11)

################################################################################
# Dependencies
################################################################################


find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        sensor_msgs
        cv_bridge
)

find_package(OpenCV REQUIRED)

find_package(TegraMM REQUIRED)
if(${TegraMM_FOUND})
    message(STATUS "Found Tegra Multimedia API")
else()
    message(STATUS "Could not find Tegra Multimedia API")
endif()

################################################################################
# Catkin Package
################################################################################


catkin_package(
        INCLUDE_DIRS
            include
        LIBRARIES
            ${PROJECT_NAME}
        CATKIN_DEPENDS
            roscpp
            roslib
            sensor_msgs
            cv_bridge
        DEPENDS
            OpenCV
)


################################################################################
# Build
################################################################################


add_library(${PROJECT_NAME} SHARED
        src/ros_tegra_utilities_lib/tegra_utilities.cpp
        src/ros_tegra_utilities_lib/jpeg_compression_node.cpp
        ${TegraMM_COMMON_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${TegraMM_LIBRARIES}
)

################################################################################
# Executables
################################################################################


add_executable(jpeg_compression_node src/jpeg_compression_node_main.cpp)
add_dependencies(jpeg_compression_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(jpeg_compression_node PUBLIC ${PROJECT_NAME})

################################################################################
# Install
################################################################################


# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
)
