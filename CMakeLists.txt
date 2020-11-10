cmake_minimum_required(VERSION 3.0.2)
project(actor2_support)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  piksi_rtk_msgs
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES actor2_support
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(piksi_odom_pub src/piksi_odom_pub.cpp)
add_dependencies(piksi_odom_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(piksi_odom_pub ${catkin_LIBRARIES})

install(DIRECTORY config/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
FILE_PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
)

install(DIRECTORY scripts/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
FILE_PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE
)

install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)