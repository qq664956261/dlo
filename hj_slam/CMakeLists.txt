#############################################################
#                                                           #
# Copyright (c) 2022, University of California, Los Angeles #
#                                                           #
# Authors: Kenny J. Chen, Brett T. Lopez                    #
# Contact: kennyjchen@ucla.edu, btlopez@ucla.edu            #
#                                                           #
#############################################################

cmake_minimum_required(VERSION 3.10.0)
project(hj_slam)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
add_definitions(-Wno-deprecated-declarations)
find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

find_package( Sophus REQUIRED )

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  message_generation
  laser_geometry
)

add_service_files(
  DIRECTORY srv
  FILES
  save_pcd.srv
  save_traj.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    nano_gicp
    nanoflann
)

include_directories(include 
${catkin_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${Sophus_INCLUDE_DIRS}
livox_ros_driver2)

include_directories("${CMAKE_SOURCE_DIR}/hj_interface/include")
include_directories("${CMAKE_SOURCE_DIR}/thirdparty/include")
link_directories("${CMAKE_SOURCE_DIR}/hj_interface/platforms/${PLATFORM}")
link_directories("${CMAKE_SOURCE_DIR}/thirdparty/platforms/${PLATFORM}")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Odometry Node
add_executable(dlo_odom_node src/hj/odom_node.cc src/hj/odom.cc src/hj/PointReader.cpp src/hj/ExtrinsicErrorTerm.cpp src/hj/scancontext.cpp)
add_dependencies(dlo_odom_node ${catkin_EXPORTED_TARGETS})
target_compile_options(dlo_odom_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlo_odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp ${CERES_LIBRARIES} ${OpenCV_LIBS} ${Sophus_LIBRARIES})

# Mapping Node
add_executable (dlo_map_node src/hj/map_node.cc src/hj/map.cc)
add_dependencies(dlo_map_node ${catkin_EXPORTED_TARGETS})
target_compile_options(dlo_map_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlo_map_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)
target_link_libraries(dlo_map_node base_framework dl)
set_target_properties(dlo_map_node PROPERTIES
LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/${INSTALL_DEVEL_PATH})


