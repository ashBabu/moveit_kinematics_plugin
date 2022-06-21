cmake_minimum_required(VERSION 3.0.2)
project(moveit_kinematics_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(
    catkin REQUIRED COMPONENTS
    moveit_core
    pluginlib
    roscpp
)

catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES moveit_kinematics_plugin
    CATKIN_DEPENDS moveit_core pluginlib roscpp
#    DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

set(LIBRARY_NAME moveit_plugin)
add_library(${LIBRARY_NAME} src/TestKinematicsPlugin.cpp)
target_link_libraries(
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES} ${LAPACK_LIBRARIES}
)

install(
    TARGETS
    ${LIBRARY_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
    FILES
    plugin_description.xml
    DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############


if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(
            catkin REQUIRED COMPONENTS
            moveit_core
            moveit_ros_planning
            roscpp
            rospy
            Eigen3
            actionlib
            moveit_ros_planning_interface
            moveit_visual_tools
            )
    catkin_add_gtest(
        analytic_ik_test
        test/analytic_ik_test.cpp
        src/TestKinematicsPlugin.cpp
    )
    catkin_package(
            CATKIN_DEPENDS
            moveit_ros_planning_interface
            moveit_visual_tools
    )

    target_link_libraries(
        analytic_ik_test
        ${catkin_LIBRARIES}
    )
    add_dependencies(
            analytic_ik_test
            ${${PROJECT_NAME}_EXPORTED_TARGETS}
            ${catkin_EXPORTED_TARGETS})
endif()

