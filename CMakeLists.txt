cmake_minimum_required(VERSION 2.8.3)
project(chesslab_setup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf2
  tf2_ros
  robotiq_2f_model
  ur_description
  ur_e_description
  roslib
  ur3ik
  downward_ros
  gazebo_ros
  aruco_ros
  #kautham
  actionlib
  actionlib_msgs
  trajectory_msgs
  control_msgs
  gazebo_ros_link_attacher
)

find_package(
   kautham REQUIRED
)

find_package(Boost REQUIRED COMPONENTS system)

if(KAUTHAM_INCLUDE_DIR)
    message(STATUS "Kautham found and its include dir is $(KAUTHAM_INCLUDE_DIR")
endif()

add_service_files(
  FILES
  setrobconf.srv
  setobjpose.srv
  attachobs2robot.srv
  dettachobs.srv
  ffplan.srv
  ik.srv
  pickAndPlace.srv
  set_point_to_point_trajectory.srv
  move_robot_trajectory.srv
  move_robot_trajectory_safe.srv
  set_goal_tolerances.srv
  getpiecepose.srv
  poseOrientGrip.srv
  userCommands.srv
  setConf.srv
)
add_message_files(
  FILES
  chessmap.msg
  iksol.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   trajectory_msgs
   control_msgs
   actionlib_msgs
   sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_basics_pkg
   CATKIN_DEPENDS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   message_runtime
   robotiq_2f_model
   ur_description
   ur_e_description
   ur3ik
   downward_ros #kautham
   gazebo_ros
   aruco_ros
        actionlib actionlib_msgs  tf2 tf2_ros trajectory_msgs control_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_executable(chesslab_setup_node src/chesslab_setup_node.cpp)
add_executable(demo_attach src/demo_attach.cpp)
add_executable(demo_detach src/demo_detach.cpp)
add_executable(demo_collision src/demo_collision.cpp)
add_executable(demo_planmotions src/demo_planmotions.cpp)
add_executable(demo_ik src/demo_ik.cpp)
add_executable(follow_traj_wrap_server src/follow_traj_wrap_server.cpp)
add_executable(follow_traj_client src/follow_traj_client.cpp)
add_executable(singlePoseOrient src/singlePoseOrient.cpp)
add_executable(poseOrientGrip src/poseOrientGrip.cpp)
add_executable(userCommands src/userCommands.cpp)
add_executable(mainNode src/mainNode.cpp)
add_executable(ur3_node src/ur3_node.cpp)

add_dependencies(chesslab_setup_node ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_attach ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_detach ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_collision ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_planmotions ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_ik ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(follow_traj_wrap_server  ${chesslab_setup_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
add_dependencies(follow_traj_client  ${chesslab_setup_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
add_dependencies(singlePoseOrient ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(poseOrientGrip ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(userCommands ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mainNode ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ur3_node ${chesslab_setup_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(chesslab_setup_node ${catkin_LIBRARIES})
target_link_libraries(demo_attach ${catkin_LIBRARIES})
target_link_libraries(demo_detach ${catkin_LIBRARIES})
target_link_libraries(demo_collision ${catkin_LIBRARIES})
target_link_libraries(demo_planmotions ${catkin_LIBRARIES})
target_link_libraries(demo_ik ${catkin_LIBRARIES})
target_link_libraries(follow_traj_wrap_server  ${catkin_LIBRARIES} )
target_link_libraries(follow_traj_client  ${catkin_LIBRARIES} )
target_link_libraries(singlePoseOrient ${catkin_LIBRARIES})
target_link_libraries(poseOrientGrip ${catkin_LIBRARIES})
target_link_libraries(userCommands ${catkin_LIBRARIES})
target_link_libraries(mainNode ${catkin_LIBRARIES})
target_link_libraries(ur3_node ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############
