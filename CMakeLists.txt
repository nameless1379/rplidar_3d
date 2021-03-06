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
  pcl_conversions
  message_generation
  angles
  cmake_modules
  tf
  tf2
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

include_directories(
  include
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
link_directories(${Eigen3_LIBRARY_DIRS})

find_package( PCL REQUIRED )

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )

  # For Use Not PreCompiled Features
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
endif()


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
  DEPENDS Boost Eigen3
)

add_executable(rplidarNode src/lidar_node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/lidar_client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})

add_executable(LaserScanGen src/laserScanGenerator.cpp)
target_link_libraries(LaserScanGen ${catkin_LIBRARIES})

add_executable(PCLGen src/PCL2Generator.cpp)
target_link_libraries(PCLGen ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${tf_LIBRARIES})

add_executable(Icp src/icp_node.cpp)
target_link_libraries(Icp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES} ${tf_LIBRARIES})

add_executable(stm32Node src/stm32_node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(stm32Node ${catkin_LIBRARIES} ${tf_LIBRARIES})
add_dependencies(stm32Node rplidar_3d_gencpp)

add_executable(stm32Cmd src/stm32_controller.cpp)
target_link_libraries(stm32Cmd ${catkin_LIBRARIES})
add_dependencies(stm32Cmd rplidar_3d_gencpp)

install(TARGETS rplidarNode rplidarNodeClient LaserScanGen PCLGen stm32Node stm32Cmd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
