cmake_minimum_required(VERSION 3.0.2)
project(pc_application)


find_package(catkin REQUIRED
    COMPONENTS
        rospy
        geometry_msgs
        )


#################
# Catkin Config #
#################

catkin_package()


#############
## Install ##
#############

install(DIRECTORY scripts launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
