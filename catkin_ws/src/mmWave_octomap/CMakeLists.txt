cmake_minimum_required(VERSION 3.5.1)

project(mmWave_octomap LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  gtrack
  serial
)

catkin_package(
   CATKIN_DEPENDS 
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    gtrack
)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  message("----------------------------------------------------------------")
  message("Debug build.")
  message("----------------------------------------------------------------")
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
  message("----------------------------------------------------------------")
  message("Release build.")
  message("----------------------------------------------------------------")
ELSE()
  message("Some other build type.")
ENDIF()

add_executable(mmWave_octomap src/tracker.cpp)
add_executable(transform_broadcaster src/transform_broadcaster.cpp)

target_include_directories(mmWave_octomap
    PUBLIC 
      $<INSTALL_INTERFACE:include>    
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${catkin_INCLUDE_DIRS}
)

target_include_directories(transform_broadcaster
    PUBLIC 
      $<INSTALL_INTERFACE:include>    
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${catkin_INCLUDE_DIRS}
)

target_link_libraries(mmWave_octomap 
  PRIVATE
    ${serial_LIBRARIES}
    ${gtrack_LIBRARIES}
    ${catkin_LIBRARIES}
)
target_link_libraries(transform_broadcaster
  PRIVATE
    ${serial_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_compile_options(mmWave_octomap PRIVATE -Wall -DMMWAVE_OCTOMAP_DEBUG) #-Wshadow -Werror -Wextra  -Wpedantic -Wconversion
set_target_properties(mmWave_octomap PROPERTIES CXX_STANDARD 11)

target_compile_options(transform_broadcaster PRIVATE -Wall) #-Wshadow -Werror -Wextra  -Wpedantic -Wconversion
set_target_properties(transform_broadcaster PROPERTIES CXX_STANDARD 11)

# catkin_add_gtest(utest tests/src/main.cpp)
