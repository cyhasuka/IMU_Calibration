cmake_minimum_required(VERSION 3.0.2)
project(art_imu_01)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
)

add_message_files(FILES
  rpy.msg
)

generate_messages(DEPENDENCIES
  std_msgs 
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  #geometry_msgs
  roscpp
  std_msgs
  message_runtime
#  INCLUDE_DIRS include

#  CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(art_imu_node src/art_imu.cpp src/serial_to_imu.cpp)
target_link_libraries(art_imu_node ${catkin_LIBRARIES})
add_dependencies(art_imu_node ${catkin_EXPORTED_TARGETS})




add_executable(ori2rpy
  src/ori2rpy.cpp 
)
add_dependencies(ori2rpy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ori2rpy
  ${catkin_LIBRARIES}
)

add_executable(sub_rpy src/sub_rpy.cpp)
target_link_libraries(sub_rpy ${catkin_LIBRARIES})
