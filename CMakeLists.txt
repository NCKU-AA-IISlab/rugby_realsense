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
  dynamic_reconfigure
)

find_package(PCL 1.7 REQUIRED)

generate_dynamic_reconfigure_options(cfg/imu.cfg)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
    dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# For nodelets
add_library(${PROJECT_NAME}  src/nodelets/PointCloudSetValid.cpp
			     src/nodelets/LaserToPointCloud.cpp  )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

# For python nodes
catkin_install_python(PROGRAMS 
	src/python/imu_node.py
	src/python/display_3D_visualization.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/python
)

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
