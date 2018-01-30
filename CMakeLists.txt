cmake_minimum_required(VERSION 2.8.3)
project(rplidar_3d)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  rosconsole
  sensor_msgs
  geometry_msgs
  message_generation
)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  stm32_cmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

add_executable(rplidarNode src/lidar_node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/lidar_client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})

add_executable(stm32Node src/stm32_node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(stm32Node ${catkin_LIBRARIES})
add_dependencies(stm32Node rplidar_3d_gencpp)

add_executable(stm32Cmd src/stm32_controller.cpp)
target_link_libraries(stm32Cmd ${catkin_LIBRARIES})
add_dependencies(stm32Cmd rplidar_3d_gencpp)

install(TARGETS rplidarNode rplidarNodeClient stm32Node stm32Cmd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
