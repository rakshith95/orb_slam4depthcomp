cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Check for c++14 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
IF(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/orb_slam2/cmake_modules)

find_package (catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
cv_bridge
image_transport
tf
sensor_msgs
dynamic_reconfigure
message_generation
# occupancy_grid_extractor
angles
)
# disable_pal_flags()

find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
     endif()
   endif()
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set (DYNAMIC_RECONFIGURE_PATH ros/config/dynamic_reconfigure.cfg)
execute_process(COMMAND chmod a+x ${DYNAMIC_RECONFIGURE_PATH}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  RESULT_VARIABLE cmd_result
  OUTPUT_VARIABLE cmd_ver)
message(STATUS "Chmod a+x the dynamic_reconfigure file")

generate_dynamic_reconfigure_options(${DYNAMIC_RECONFIGURE_PATH})

set(LIBS_ORBSLAM
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/lib/libg2o.so
)

set(LIBS_ROS
  ${PROJECT_SOURCE_DIR}/orb_slam2/lib/lib${PROJECT_NAME}.so
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime occupancy_grid_extractor
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

#include_directories(
#  SYSTEM
#  ${PROJECT_SOURCE_DIR}/orb_slam2
#  ${PROJECT_SOURCE_DIR}/orb_slam2/include
#  ${PROJECT_SOURCE_DIR}/ros/include
#  ${EIGEN3_INCLUDE_DIR}
#  ${catkin_INCLUDE_DIRS}
#)

include_directories(
  ${PROJECT_SOURCE_DIR}/orb_slam2
  ${PROJECT_SOURCE_DIR}/orb_slam2/include
  ${PROJECT_SOURCE_DIR}/ros/include
)

include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


include(${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam2/lib)

add_library(${PROJECT_NAME} SHARED
  orb_slam2/src/System.cc
  orb_slam2/src/Tracking.cc
  orb_slam2/src/LocalMapping.cc
  orb_slam2/src/LoopClosing.cc
  orb_slam2/src/ORBextractor.cc
  orb_slam2/src/ORBmatcher.cc
  orb_slam2/src/FrameDrawer.cc
  orb_slam2/src/Converter.cc
  orb_slam2/src/MapPoint.cc
  orb_slam2/src/KeyFrame.cc
  orb_slam2/src/Map.cc
  orb_slam2/src/Optimizer.cc
  orb_slam2/src/PnPsolver.cc
  orb_slam2/src/Frame.cc
  orb_slam2/src/KeyFrameDatabase.cc
  orb_slam2/src/Sim3Solver.cc
  orb_slam2/src/Initializer.cc
)
add_dependencies (${PROJECT_NAME} g2o DBoW2)

target_link_libraries(${PROJECT_NAME} ${LIBS_ORBSLAM})

# map serialization addition - library boost serialization 
message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

add_executable (${PROJECT_NAME}_mono
  ros/src/MonoNode.cc
  ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_mono ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_mono ${LIBS_ROS})

add_executable (${PROJECT_NAME}_stereo
  ros/src/StereoNode.cc
  ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_stereo ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_stereo ${LIBS_ROS})

add_executable (${PROJECT_NAME}_rgbd
  ros/src/RGBDNode.cc
  ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_rgbd ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_rgbd ${LIBS_ROS})

install(TARGETS ${PROJECT_NAME}_mono ${PROJECT_NAME}_stereo ${PROJECT_NAME}_rgbd ${PROJECT_NAME} DBoW2 g2o
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir ros/launch/ ros/config/ orb_slam2/config/ orb_slam2/Vocabulary/)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
