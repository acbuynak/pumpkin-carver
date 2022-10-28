cmake_minimum_required(VERSION 3.0.2)
project(pc_canvas_locking)


find_package(catkin REQUIRED
    COMPONENTS
        rospy
        tf2
        geometry_msgs
        message_runtime
        )

##################
# Custom Msg/Srv #
##################

# Custom Services
add_service_files(
  FILES
  WorkpieceLocking.srv
)

# Generate custom service message
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#################
# Catkin Config #
#################

catkin_package(CATKIN_DEPENDS message_runtime)

#############
## Install ##
#############

install(PROGRAMS nodes/server
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
