cmake_minimum_required(VERSION 2.8.3)
project(sparse_stereo)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  octomap_ros
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  octomap_server
  eigen_conversions
  cv_bridge
  sensor_msgs
  std_msgs
  pips_trajectory_msgs
  sparse_stereo_msgs
  image_transport
  message_filters
  message_generation
  image_geometry
  dynamic_reconfigure
  turtlebot_trajectory_functions
)

find_package(Boost    REQUIRED COMPONENTS thread system filesystem)
find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(OpenMP)
# find_package(Eigen3   REQUIRED)

# customize eigen dir
# find_package(Eigen3 3.1.0 REQUIRED)
set( EIGEN3_INCLUDE_DIR /opt/eigen33/include/eigen3/ )
# set( EIGEN3_INCLUDE_DIR /usr/include/eigen3 )
MESSAGE("EIGEN include dir: " ${EIGEN3_INCLUDE_DIR})
# MESSAGE("EIGEN libs: " ${EIGEN3_LIBS})

# customize pangolin dir
# set(Pangolin_DIR /opt/Pangolin/lib/cmake/Pangolin/)
# find_package(Pangolin REQUIRED)
set(Pangolin_INCLUDE_DIRS /opt/Pangolin/include/)
set(Pangolin_LIBRARIES /opt/Pangolin/lib/libpangolin.so)
MESSAGE("Pangolin include dir: " ${Pangolin_INCLUDE_DIRS})


# customize openblas dir
set(OpenBLAS_LIBRARIES /opt/OpenBLAS/lib/libopenblas.so)
MESSAGE("OpenBLAS libs:" ${OpenBLAS_LIBRARIES})

# customize armadillo dir
set(ARMADILLO_INCLUDE_DIR /opt/armadillo/include)
set(ARMADILLO_LIBRARIES /opt/armadillo/lib/libarmadillo.so)
MESSAGE("ARMADILLO libs: " ${ARMADILLO_LIBRARIES})

# customize gtest dir
set(GTest_INCLUDE_DIR /usr/src/gtest/src)
set(GTest_LIBRARIES /usr/src/gtest/libgtest.a)
MESSAGE("gtest libs: " ${GTest_LIBRARIES})

find_package(GFlags REQUIRED)

#set(GF_ORB_SLAM2_LIBRARIES ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/lib/libgf_orb_slam2.so)
#set(GF_ORB_SLAM2_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/include/)
#set(LIBS
#        GL
#        glut
#        GLU
#        ${GFLAGS_LIBRARIES}
#        ${OpenCV_LIBS}
#        # ${EIGEN3_LIBS}
#        ${Pangolin_LIBRARIES}
#        ${OpenBLAS_LIBRARIES}
#        ${ARMADILLO_LIBRARIES}
#        ${Boost_LIBRARIES}
#        ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
#        ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
#        #trajectory_state_predictor
#    )
#SET(SLAM_PLUS_PLUS_LIBRARIES
#        ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/SLAM++/lib/libeigen.so
#        ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/SLAM++/lib/libcxsparse.so
#        ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/SLAM++/lib/libcsparse.so
#        ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/SLAM++/lib/libcholmod.so
#        ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/SLAM++/lib/libcamd_dlong.so
#        ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/SLAM++/lib/libslampp_base.so
#    )    




MESSAGE(STATUS "opencv_version='${OpenCV_VERSION}'")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -std=c++11 ${OpenMP_CXX_FLAGS} -O3")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread")

generate_dynamic_reconfigure_options(
  cfg/SparseStereoTester.cfg
  )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS pcl_ros octomap_ros dynamic_reconfigure tf2_ros turtlebot_trajectory_functions pips_trajectory_msgs sparse_stereo_msgs #Note: the generic parts of the last 2 should probably be moved to a separate package
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/stvo/3rdparty/DBoW2/DBoW2/
  ${PROJECT_SOURCE_DIR}/stvo/3rdparty/line_descriptor/include/
  ${PROJECT_SOURCE_DIR}/stvo/include/
#  ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/
#  ${PROJECT_SOURCE_DIR}/../GF_ORB_SLAM2/Thirdparty/SLAM++/include
#  ${GF_ORB_SLAM2_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIR}
  ${GTest_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
message(${catkin_INCLUDE_DIRS})

# Note: Do not compile with C++11: causes segfaults

## Declare a C++ library
# add_library(baseline_testing
#   src/${PROJECT_NAME}/baseline_testing.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(baseline_testing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_library(local_octomap_generator src/local_octomap_generator.cpp)

add_library(pips_sparse_stereo src/pips_sparse_stereo_processor.cpp)
add_dependencies(pips_sparse_stereo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_library(orb_slam_sparse_stereo_processor src/orb_slam_sparse_stereo_processor.cpp)
#add_dependencies(orb_slam_sparse_stereo_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_library(orb_slam_rgbd_processor src/orb_slam_rgbd_processor.cpp)
#add_dependencies(orb_slam_rgbd_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_library(visual_servo_core src/visual_servo_core.cpp)
# add_dependencies(visual_servo_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_library(visual_servo_navigator src/visual_servo_navigator.cpp)
# add_dependencies(visual_servo_navigator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_library(cubic_spline_interpolator src/cubic_spline_interpolator.cpp)
# add_dependencies(cubic_spline_interpolator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(local_octomap_generator_node src/local_octomap_generator_node.cpp)

add_executable(pips_sparse_stereo_node src/pips_sparse_stereo_node.cpp)

#add_executable(orb_slam_sparse_stereo_node src/orb_slam_sparse_stereo_node.cpp)

#add_executable(orb_slam_rgbd_node src/orb_slam_rgbd_node.cpp)

add_executable(depth_tester src/depth_image_tester.cpp)

add_executable(image_blur src/image_blur_node.cpp)

# add_executable(visual_servo_navigator_node src/visual_servo_navigator_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(baseline_testing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(local_octomap_generator
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(pips_sparse_stereo
  ${PROJECT_SOURCE_DIR}/stvo/lib/libstvo.so
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#target_link_libraries(orb_slam_sparse_stereo_processor
#  GL
#  glut
#  GLU
#  ${LIBS}
#  ${GF_ORB_SLAM2_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(orb_slam_rgbd_processor
#  GL
#  glut
#  GLU
#  ${LIBS}
#  ${GF_ORB_SLAM2_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

target_link_libraries(local_octomap_generator_node
  local_octomap_generator
  ${catkin_LIBRARIES}
)

target_link_libraries(pips_sparse_stereo_node
  pips_sparse_stereo
  ${PROJECT_SOURCE_DIR}/stvo/3rdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/stvo/3rdparty/line_descriptor/lib/liblinedesc.so
  ${catkin_LIBRARIES}
)

#target_link_libraries(orb_slam_sparse_stereo_node
#  orb_slam_sparse_stereo_processor
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(orb_slam_rgbd_node
#  orb_slam_rgbd_processor
#  ${catkin_LIBRARIES}
#)

target_link_libraries(depth_tester
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(image_blur
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

# target_link_libraries(cubic_spline_interpolator
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(visual_servo_core
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(visual_servo_navigator
#   visual_servo_core
#   cubic_spline_interpolator
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(visual_servo_navigator_node
#   visual_servo_navigator
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS baseline_testing baseline_testing_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_baseline_testing.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
