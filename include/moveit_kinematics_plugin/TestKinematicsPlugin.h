#ifndef _INHOUSE_MOVEIT_KINEMATICS_PLUGIN_H_
#define _INHOUSE_MOVEIT_KINEMATICS_PLUGIN_H_

#include <numeric>
#include <ros/ros.h>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <pluginlib/class_list_macros.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit/kinematics_base/kinematics_base.h>

namespace inhouse_moveit_kinematics_plugin
{
class TestKinematicsPlugin : public kinematics::KinematicsBase
{
public:

    TestKinematicsPlugin();

    ~TestKinematicsPlugin();

    
    bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                    const std::string& base_frame, const std::vector<std::string>& tip_frames,
                    double search_discretization) override;

    // Returns the IK solution that is within joint limits closest to ik_seed_state
    bool getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                  std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                  const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                       std::vector<std::vector<double>>& solutions, kinematics::KinematicsResult& result,
                       const kinematics::KinematicsQueryOptions& options) const override;

    bool searchPositionIK(
            const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
            std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
            const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(
            const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
            const std::vector<double>& consistency_limits, std::vector<double>& solution,
            moveit_msgs::MoveItErrorCodes& error_code,
            const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(
            const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
            std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
            const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(
            const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
            const std::vector<double>& consistency_limits, std::vector<double>& solution,
            const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
            const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                       std::vector<geometry_msgs::Pose>& poses) const override;

private:
    std::string name_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    double a1_, a2_, a3_, a4_;  // 

    const robot_state::JointModelGroup* joint_model_group_;
    bool initialized_;  ///< Internal variable that indicates whether solver is configured and ready

    unsigned int dimension_;                        ///< Dimension of the group
    KDL::Chain kdl_chain_;
    moveit_msgs::KinematicSolverInfo solver_info_;

    Eigen::VectorXd joint_min_, joint_max_;
    robot_state::RobotStatePtr state_;

    const std::vector<std::string>& getJointNames() const override
    {
        return joint_names_;
    }
    const std::vector<std::string>& getLinkNames() const override
    {
        return link_names_;
    }

    std::vector<double> getNearestSolutionToSeed(const std::vector<double> &ik_seed_state,
                                                 const std::vector<std::vector<double>> &solutions) const;
    
    bool getIKSolutions(const geometry_msgs::Pose &ik_pose,
                        std::vector<std::vector<double>> &solutions) const;
    
    bool isSolutionValid(const double &q) const;

    moveit::core::RobotStatePtr robot_state_;
    std::shared_ptr<ros::ServiceClient> ik_service_client_;

    uint num_joints_;

};
}

#endif //_INHOUSE_MOVEIT_KINEMATICS_PLUGIN_H_
