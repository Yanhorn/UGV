cmake_minimum_required(VERSION 2.8.3)
project(neuvition_driver)

set(OpenCV_DIR /usr/share/OpenCV)

find_package(OpenCV 3 REQUIRED)

set(${PROJECT_NAME}_CATKIN_DEPS 
    dynamic_reconfigure
    message_generation
    nodelet
    roscpp
    tf
    pcl_conversions
    pcl_ros
    std_msgs
    sensor_msgs
    image_transport
    cv_bridge
    )

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})


add_message_files(
  FILES
  test.msg
  # Message2.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# This driver uses Boost::thread
#find_package(Boost REQUIRED COMPONENTS thread)
# This driver uses PCL
find_package(PCL REQUIRED)

add_compile_options(-std=c++11 -O3)
add_compile_options(-mcmodel=medium)
# libneuvsdk

find_package(OpenCV REQUIRED imgcodecs)

#${Boost_INCLUDE_DIRS}
include_directories(include  ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Generate dynamic_reconfigure server
generate_dynamic_reconfigure_options(cfg/NeuvitionNode.cfg)

# objects needed by other ROS packages that depend on this one
catkin_package(CATKIN_DEPENDS message_runtime ${${PROJECT_NAME}_CATKIN_DEPS} INCLUDE_DIRS include  )

# compile the driver and input library
add_subdirectory(src)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_neuvition.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)


add_executable(sub_node src/icp_node.cpp)

target_link_libraries(sub_node
        ${catkin_LIBRARIES}
)

add_executable(pose_node src/pose_node.cpp)

target_link_libraries(pose_node
        ${catkin_LIBRARIES}
)



