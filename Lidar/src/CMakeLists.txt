# build the driver node

add_executable(neuvition_node neuvition_node.cc driver.cc )

add_dependencies(neuvition_node neuvition_driver_gencfg)
set  (LIBCURL ${PROJECT_SOURCE_DIR}/lib/libcurl.so)

set(OpenCV_DIR /usr/share/OpenCV)
find_package(OpenCV 3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

#${Boost_LIBRARIES}
target_link_libraries(neuvition_node
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${LIBCURL}
  ${PROJECT_SOURCE_DIR}/lib/libneusdk_boost.so
  ${PROJECT_SOURCE_DIR}/lib/x86/libwoodpecker_arm.so
)

#${Boost_LIBRARIES}
# build the nodelet version
add_library(neuvdrv_nodelet nodelet.cc driver.cc  )
add_dependencies(neuvdrv_nodelet neuvition_driver_gencfg)
target_link_libraries(neuvdrv_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${LIBCURL}
)

# install runtime files
install(TARGETS neuvition_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)

install(TARGETS neuvdrv_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(PROGRAMS ${PROJECT_SOURCE_DIR}/lib/x86/libneusdk.so.3.0.2
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS neuvition_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink
                        ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/libneusdk.so.3.0.2
						${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libneusdk_boost.so)"
)


