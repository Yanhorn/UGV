cmake_minimum_required(VERSION 2.8.3)
project(path_navi)

find_package(catkin REQUIRED
  COMPONENTS
  nav_core
  nav_msgs
  roscpp
  actionlib_msgs
  tf
  actionlib
)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_action_files(
  DIRECTORY action
  FILES
  PathNavi.action
)

# add_message_files(
# FILES
# chassis_speed.msgs
# )
generate_messages(
  DEPENDENCIES actionlib_msgs nav_msgs std_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  nav_msgs
  roscpp
  actionlib_msgs
)
#add_library(point_navi src/point_navi_server.cpp)
#add_executable(path_navi_client src/path_navi_client.cpp)
#target_link_libraries(path_navi_client ${catkin_LIBRARIES})

add_executable(path_navi_server src/path_navi_server.cpp src/pure_persuit.cpp)
target_link_libraries(path_navi_server ${catkin_LIBRARIES}  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libyaml-cpp.so)


## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS path_navi_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS path_navi_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch action msgs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
