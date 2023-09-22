cmake_minimum_required(VERSION 3.2)
project(graph_rviz_plugin)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  class_loader
  cv_bridge
  roscpp
  rqt_gui
  rviz
  sensor_msgs
)

## Find Qt5
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets 5.7 REQUIRED)
find_package(Qt5PrintSupport 5.7 REQUIRED)

set(
  PROJECT_QT_PANELS_TARGETS
  ${PROJECT_NAME}_line
  ${PROJECT_NAME}_histogram
)

set(
  PROJECT_TARGETS
  ${PROJECT_QT_PANELS_TARGETS}
  ${PROJECT_NAME}_graph_settings
  ${PROJECT_NAME}_qcustomplot
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_settings
  ${PROJECT_NAME}_topic_color
  ${PROJECT_NAME}_topic_style
  ${PROJECT_NAME}_topics_data
)

################################################
## Declare ROS messages, services and actions ##
################################################

#####################################################
## Declare ROS dynamic regraph_settings parameters ##
#####################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_TARGETS}
  CATKIN_DEPENDS
  cv_bridge
  roscpp
  rqt_gui
  rviz
  sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## QCustomPlot
add_library(
  ${PROJECT_NAME}_qcustomplot
  include/${PROJECT_NAME}/qcustomplot.h
  src/qcustomplot.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_qcustomplot
  Qt5::Widgets
  Qt5::PrintSupport
)

## TopicsData
add_library(
  ${PROJECT_NAME}_topics_data
  include/${PROJECT_NAME}/topic_data.hpp
  src/topic_data.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_topics_data
  Qt5::Widgets
)

## TopicColor
add_library(
  ${PROJECT_NAME}_topic_color
  include/${PROJECT_NAME}/topic_color.hpp
  src/topic_color.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_topic_color
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_topic_color
  ${catkin_EXPORTED_TARGETS}
)

## TopicStyle
add_library(
        ${PROJECT_NAME}_topic_style
        include/${PROJECT_NAME}/topic_style.hpp
        src/topic_style.cpp
)
target_link_libraries(
        ${PROJECT_NAME}_topic_style
        Qt5::Widgets
)
add_dependencies(
        ${PROJECT_NAME}_topic_style
        ${catkin_EXPORTED_TARGETS}
)

## SelectionTopics
add_library(
  ${PROJECT_NAME}_selection_topics
  include/${PROJECT_NAME}/selection_topics.hpp
  src/selection_topics.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_topics_data
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_topics_data
  ${catkin_EXPORTED_TARGETS}
)

## Graph settings
add_library(
  ${PROJECT_NAME}_graph_settings
  include/${PROJECT_NAME}/graph_settings.hpp
  src/graph_settings.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_graph_settings
  ${PROJECT_NAME}_topics_data
  ${PROJECT_NAME}_topic_color
  ${PROJECT_NAME}_topic_style
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_graph_settings
  ${PROJECT_NAME}_topics_data
  ${PROJECT_NAME}_topic_color
  ${PROJECT_NAME}_topic_style
  ${catkin_EXPORTED_TARGETS}
)

## Settings
add_library(
  ${PROJECT_NAME}_settings
  include/${PROJECT_NAME}/settings.hpp
  src/settings.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_settings
  Qt5::Widgets
)
add_dependencies(
  ${PROJECT_NAME}_settings
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel
add_library(
  ${PROJECT_NAME}_line
  include/${PROJECT_NAME}/line_panel.hpp
  src/line_panel.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_line
  ${PROJECT_NAME}_graph_settings
  ${PROJECT_NAME}_qcustomplot
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_settings
  ${PROJECT_NAME}_topics_data
  ${PROJECT_NAME}_topic_color
  ${PROJECT_NAME}_topic_style
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_line
  ${PROJECT_NAME}_graph_settings
  ${PROJECT_NAME}_qcustomplot
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_settings
  ${PROJECT_NAME}_topics_data
  ${PROJECT_NAME}_topic_color
  ${PROJECT_NAME}_topic_style
  ${catkin_EXPORTED_TARGETS}
)

## Qt panel
add_library(
  ${PROJECT_NAME}_histogram
  include/${PROJECT_NAME}/histogram_panel.hpp
  src/histogram_panel.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_histogram
  ${PROJECT_NAME}_graph_settings
  ${PROJECT_NAME}_qcustomplot
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_settings
  ${PROJECT_NAME}_topics_data
  ${PROJECT_NAME}_topic_color
  ${PROJECT_NAME}_topic_style
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_histogram
  ${PROJECT_NAME}_graph_settings
  ${PROJECT_NAME}_qcustomplot
  ${PROJECT_NAME}_selection_topics
  ${PROJECT_NAME}_settings
  ${PROJECT_NAME}_topics_data
  ${PROJECT_NAME}_topic_color
  ${PROJECT_NAME}_topic_style
  ${catkin_EXPORTED_TARGETS}
)

# Needed by Qt / RViz / ROS
class_loader_hide_library_symbols(
  ${PROJECT_QT_PANELS_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
  TARGETS
  ${PROJECT_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES
  plugin.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS
  scripts/subtopic_repub.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############
