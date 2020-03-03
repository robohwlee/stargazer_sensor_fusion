cmake_minimum_required(VERSION 2.8.3)
project(stargazer)

find_package(catkin REQUIRED
  std_msgs
  geometry_msgs
  message_generation
  )

add_message_files(
  FILES
    MarkerPose.msg
    MarkerPoses.msg
    MarkerRawPose.msg
    MarkerRawPoses.msg
 )

catkin_python_setup()

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
  	geometry_msgs
    message_runtime
)

install(PROGRAMS scripts/stargazer_publisher.py
    DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
