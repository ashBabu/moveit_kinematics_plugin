#include <vector>
#include <gtest/gtest.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit_kinematics_plugin/TestKinematicsPlugin.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

TEST(TESTSuite, AnalyticIkTest)
{
    inhouse_moveit_kinematics_plugin::TestKinematicsPlugin sak;

    static const std::string PLANNING_GROUP = "my_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_state::RobotState start_state(*move_group.getCurrentState());

    geometry_msgs::Pose ik_pose;
    std::vector<double> ik_seed_state, solution, solutionMoveit;
    moveit_msgs::MoveItErrorCodes error_code;
    kinematics::KinematicsQueryOptions options;
    ik_pose.position.x = 0.1; ik_pose.position.y = 0.25; ik_pose.position.z = 1.1;
//    ik_pose.position.x = 0.45; ik_pose.position.y = -0.25; ik_pose.position.z = -0.40;
    ik_seed_state.push_back(0.2); ik_seed_state.push_back(0.3);
    ik_seed_state.push_back(0.4); ik_seed_state.push_back(0.5);

    move_group.setPositionTarget(ik_pose.position.x,
                                 ik_pose.position.y,
                                 ik_pose.position.z);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(!success)
            std::cerr << "No joint plan found" << std::endl;
        else
        move_group.execute(my_plan); //blocking
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inhouse_analytic_plugin");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

