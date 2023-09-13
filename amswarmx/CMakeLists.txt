cmake_minimum_required(VERSION 3.5)
project(amswarmx)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -fopenmp -lpthread -ljsoncpp")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

link_libraries(stdc++fs)
include_directories(
include
${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  roslib
  visualization_msgs
  geometry_msgs
  std_msgs
  # pcl_ros
)

# find_package(CGAL REQUIRED)
# include_directories(${CGAL_INCLUDE_DIRS})
# if (CGAL_FOUND)
#   message(STATUS "Package 'CGAL' found.")
# else ()
#   message(FATAL_ERROR "Package 'CGAL' not found.")
# endif ()


# find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# if (PCL_FOUND)
#   message(STATUS "Package 'PCL' found.")
# else ()
#   message(FATAL_ERROR "Package 'PCL' not found.")
# endif ()


find_package(decomp_ros_utils REQUIRED)
include_directories(${decomp_ros_utils_INCLUDE_DIRS})
if (decomp_ros_utils_FOUND)
  message(STATUS "Package 'decomp_ros_utils' found.")
else ()
  message(FATAL_ERROR "Package 'decomp_ros_utils' not found.")
endif ()

find_package(decomp_ros_msgs REQUIRED)
if (decomp_ros_msgs_FOUND)
  message(STATUS "Package 'decomp_ros_msgs' found.")
else ()
  message(FATAL_ERROR "Package 'decomp_ros_msgs' not found.")
endif ()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
if (Eigen3_FOUND)
  message(STATUS "Package 'Eigen3' found.")
else ()
  message(FATAL_ERROR "Package 'Eigen3' not found.")
endif ()


find_package(jsoncpp REQUIRED)
if (jsoncpp_FOUND)
  message(STATUS "Package 'jsoncpp' found.")
else ()
  message(FATAL_ERROR "Package 'jsoncpp' not found.")
endif ()


find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})
if (dynamicEDT3D_FOUND)
  message(STATUS "Package 'dynamicEDT3D' found.")
else ()
  message(FATAL_ERROR "Package 'dynamicEDT3D' not found.")
endif ()

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
if (octomap_FOUND)
  message(STATUS "Package 'octomap' found.")
else ()
  message(FATAL_ERROR "Package 'octomap' not found.")
endif ()

find_package(jps3d REQUIRED)
include_directories(${JPS3D_INCLUDE_DIRS})
if (jps3d_FOUND)
  message(STATUS "Package 'jps3d' found.")
else ()
  message(FATAL_ERROR "Package 'jps3d' not found.")
endif ()

find_package(decomp_util REQUIRED)
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})
if (decomp_util_FOUND)
  message(STATUS "Package 'decomp_util' found.")
else ()
  message(FATAL_ERROR "Package 'decomp_util' not found.")
endif ()

catkin_package(CATKIN_DEPENDS)

# AM 
add_library(lib_am_swarm 
              src/amswarmx/obstacles_utils.cpp 
              src/amswarmx/run_grid_planner.cpp
              src/amswarmx/run_trajectory_optimizer.cpp
              src/amswarmx/select_anchor_goal.cpp
              src/amswarmx/solve_polar_var.cpp
              src/amswarmx/solve_position_var.cpp
              src/amswarmx/trajectory_utils.cpp
              src/runner/run_am_swarm.cpp)
add_executable(swarm_am_nav src/main_am_swarm.cpp)
target_link_libraries(swarm_am_nav 
                      lib_am_swarm 
                      yaml-cpp 
                      jsoncpp 
                      ${DECOMP_ROS_UTILS_INCLUDE_DIRS} 
                      ${JPS3D_LIBRARIES} 
                      ${catkin_LIBRARIES} 
                      ${DECOMP_UTIL_INCLUDE_DIRS})
