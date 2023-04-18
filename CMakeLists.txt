cmake_minimum_required(VERSION 2.8.3)
project(nao_robot_study)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ros_tcp_endpoint
  message_generation
)

add_message_files(
    FILES
    GameState.msg
    TimeState.msg
    GameUpdate.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
    ros_tcp_endpoint
    message_runtime)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nao_study.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
