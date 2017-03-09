cmake_minimum_required(VERSION 2.8.3)
project(rugby_realsense)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  sensor_msgs
  pcl_conversions
  pcl_msgs
  laser_geometry
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}  src/nodelets/PointCloudSetValid.cpp
			     src/nodelets/LaserToPointCloud.cpp  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
