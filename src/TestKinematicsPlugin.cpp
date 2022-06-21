#include <inhouse_moveit_kinematics_plugin/TestKinematicsPlugin.h>
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(inhouse_moveit_kinematics_plugin::TestKinematicsPlugin, kinematics::KinematicsBase);

using namespace inhouse_moveit_kinematics_plugin;

TestKinematicsPlugin::TestKinematicsPlugin() : initialized_(false), name_("InHouse"), num_joints_(4)
{
    std::cout<<"Initializing IN-HOUSE-ANALYTIC-KINEMATIC-PLUGIN"<<std::endl;
    a1_ = 0.2; a2_ = 0.25; a3_ = 0.1415; a4_ = -0.175;   // wrt base
}

TestKinematicsPlugin::~TestKinematicsPlugin()
{
    std::cout<<"destroyed"<<std::endl;
}


bool TestKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                         const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                         double search_discretization)
{
    if (tip_frames.size() != 1)
    {
        ROS_ERROR_NAMED(name_, "Expecting exactly one tip frame.");
        return false;
    }
    std::cout << "GroupName: " << group_name << " base_frmae: " << base_frame << " TipFrame: " << tip_frames[0] << std::endl;

    // TODO findout how to modify search discretization and use in LinearSpacedArray()
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
//    base_frame_ = "base_link";

    if (!robot_model.hasLinkModel(tip_frames_[0]))
        ROS_ERROR_STREAM_NAMED(name_, "tip frame '" << tip_frames_[0] << "' does not exist.");
    if (!robot_model.hasLinkModel(base_frame_))
        ROS_ERROR_STREAM_NAMED(name_, "base_frame '" << base_frame_ << "' does not exist.");

    joint_model_group_ = robot_model_->getJointModelGroup(group_name);
    if (!joint_model_group_)
    {
        ROS_ERROR_STREAM_NAMED(name_, "Unknown planning group: " << group_name);
        return false;
    }
    std::cout << std::endl << "Joint Model Variable Names: ------------------------------------------- " << std::endl;
    const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
    std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << std::endl;

    // Get the dimension of the planning group
    dimension_ = joint_model_group_->getVariableCount();
    ROS_INFO_STREAM_NAMED(name_, "Dimension planning group '"
            << group_name << "': " << dimension_
            << ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
            << ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());


    if (!joint_model_group_->isChain())
    {
        ROS_ERROR_NAMED("InHouse Kinematic Plugin", "Group '%s' is not a chain", group_name.c_str());
        return false;
    }
    if (!joint_model_group_->isSingleDOFJoints())
    {
        ROS_ERROR_NAMED("InHouse Kinematic Plugin", "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
        return false;
    }

    KDL::Tree kdl_tree;

    if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), kdl_tree))
    {
        ROS_ERROR_NAMED("InHouse Kinematic Plugin", "Could not initialize tree object");
        return false;
    }
    if (!kdl_tree.getChain(base_frame_, getTipFrame(), kdl_chain_))
    {
        ROS_ERROR_NAMED("InHouse Kinematic Plugin", "Could not initialize chain object");
        return false;
    }

//    dimension_ = joint_model_group_->getActiveJointModels().size() + joint_model_group_->getMimicJointModels().size();
    for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
    {
        if (joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
            joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
        {
            solver_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
            const std::vector<moveit_msgs::JointLimits>& jvec =
                    joint_model_group_->getJointModels()[i]->getVariableBoundsMsg();
            solver_info_.limits.insert(solver_info_.limits.end(), jvec.begin(), jvec.end());
        }
    }

    if (!joint_model_group_->hasLinkModel(getTipFrame()))
    {
        ROS_ERROR_NAMED("InHouse Kinematic Plugin", "Could not find tip name in joint group '%s'", group_name.c_str());
        return false;
    }

    solver_info_.link_names.push_back(getTipFrame());
    std::string ik_service_name;
//    lookupParam("kinematics_solver_service_name", ik_service_name, std::string("solve_ik"));
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

//    // Create the ROS service client
//    ros::NodeHandle nonprivate_handle("");
//    ik_service_client_ = std::make_shared<ros::ServiceClient>(
//            nonprivate_handle.serviceClient<moveit_msgs::GetPositionIK>(ik_service_name));
/*

    if (!ik_service_client_->waitForExistence(ros::Duration(0.1)))  // wait 0.1 seconds, blocking
        ROS_WARN_STREAM_NAMED("srv",
                              "Unable to connect to ROS service client with name: " << ik_service_client_->getService());
    else
        ROS_INFO_STREAM_NAMED("srv", "Service client started with ROS service name: " << ik_service_client_->getService());
*/

    joint_min_.resize(solver_info_.limits.size());
    joint_max_.resize(solver_info_.limits.size());

    for (unsigned int i = 0; i < solver_info_.limits.size(); i++)
    {
        joint_min_(i) = solver_info_.limits[i].min_position;
        joint_max_(i) = solver_info_.limits[i].max_position;
    }

//    state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    ROS_DEBUG_STREAM_NAMED(name_, "Registering joints and links");
    const moveit::core::LinkModel* link = robot_model_->getLinkModel(tip_frames_[0]);
    const moveit::core::LinkModel* base_link = robot_model_->getLinkModel(base_frame_);
    while (link && link != base_link)
    {
        ROS_DEBUG_STREAM_NAMED(name_, "Link " << link->getName());
        link_names_.push_back(link->getName());
        const moveit::core::JointModel* joint = link->getParentJointModel();
        if (joint->getType() != joint->UNKNOWN && joint->getType() != joint->FIXED && joint->getVariableCount() == 1)
        {
            ROS_DEBUG_STREAM_NAMED(name_, "Adding joint " << joint->getName());
            joint_names_.push_back(joint->getName());
        }
        link = link->getParentLinkModel();
    }
    initialized_ = true;
    ROS_DEBUG_NAMED(name_, " solver initialized");
    return true;
}

bool TestKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                            const std::vector<double> &joint_angles,
                                            std::vector<geometry_msgs::Pose> &poses) const
{
    double d = joint_angles[0], q1 = joint_angles[1], q2 = joint_angles[2], q3 = joint_angles[3];

    if (d < 0 || d > 0.365) {
        ROS_ERROR("Prismatic joint out of bounds");
        return false;
    }
    if (q1 < -M_PI_2 || q1 > M_PI_2) {
        ROS_ERROR("Joint 1 out of bounds");
        return false;
    }
    if (q2 < -M_PI_2 || q2 > M_PI_2) {
        ROS_ERROR("Joint 2 out of bounds");
        return false;
    }
    if (q3 < -M_PI_2 || q3 > M_PI_2) {
        ROS_ERROR("Joint 3 out of bounds");
        return false;
    }

    geometry_msgs::Pose pose;
    pose.position.x = a1_ * cos(q1) + a1_ * cos(q1 - q2) + a2_ * cos(q1 - q2 + q3) + a3_;
    pose.position.y = a1_ * sin(q1) + a1_ * sin(q1 - q2) + a2_ * sin(q1 - q2 + q3);
    pose.position.z = a4_ - d;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, M_PI + q1 - q2 + q3);
    pose.orientation.x = myQuaternion.getX();
    pose.orientation.y = myQuaternion.getY();
    pose.orientation.z = myQuaternion.getZ();
    pose.orientation.w = myQuaternion.getW();
    poses.push_back(pose);
    return true;
}

bool TestKinematicsPlugin::getIKSolutions(const geometry_msgs::Pose &ik_pose,
                                             std::vector<std::vector<double>> &solutions) const
{
    /// Some solution finding details omitted
    if (solutions.empty())
        return false;
    else
        return true;
}

std::vector<double> TestKinematicsPlugin::getNearestSolutionToSeed(const std::vector<double> &ik_seed_state,
                                                 const std::vector<std::vector<double>> &solutions) const;
{
    // 1. Find distance between seed_state and each of the solutions
    // 2. return the solution with least distance
}

bool TestKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                        std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
    std::cout << "GET POSITION IK 1" << std::endl;
    std::cout<< " z: " << ik_pose.position.z <<
             " x: " << ik_pose.position.x <<
             " y: " << ik_pose.position.y <<
             std::endl;

    std::vector<double> consistency_limits;
    // limit search to a single attempt by setting a timeout of zero
    return searchPositionIK(ik_pose, ik_seed_state, 0.0, consistency_limits, solution, IKCallbackFn(), error_code,
                            options);
}

bool TestKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                            const std::vector<double> &ik_seed_state,
                                            std::vector<std::vector<double>> &solutions,
                                            kinematics::KinematicsResult &result,
                                            const kinematics::KinematicsQueryOptions &options) const
{
    ROS_DEBUG_STREAM_NAMED(name_, "getPositionIK with multiple solutions");
    std::cout << "GET POSITION IK 2" << std::endl;
    if (ik_poses.empty())
    {
        ROS_ERROR_NAMED(name_, "ik_poses is empty");
        result.kinematic_error = kinematics::KinematicErrors::EMPTY_TIP_POSES;
        return false;
    }
    if (ik_poses.size() > 1)
    {
        ROS_ERROR_NAMED(name_, "ik_poses contains multiple entries, only one is allowed");
        result.kinematic_error = kinematics::KinematicErrors::MULTIPLE_TIPS_NOT_SUPPORTED;
        return false;
    }
    getIKSolutions(ik_poses[0], solutions);
    if (solutions.empty())
    {
        result.kinematic_error = kinematics::KinematicErrors::NO_SOLUTION;
        return false;
    }
    else
    {
        result.kinematic_error = kinematics::KinematicErrors::OK;
        return true;
    }
}
//1
bool TestKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                               double timeout, std::vector<double> &solution,
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
{
    std::cout<<"In SearchPositionIK 1"<< std::endl;
    std::cout<< " z: " << ik_pose.position.z <<
                " x: " << ik_pose.position.x <<
                " y: " << ik_pose.position.y <<
                std::endl;
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution,
                            IKCallbackFn(), error_code, options);
}
//2
bool TestKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                               double timeout, const std::vector<double> &consistency_limits,
                                               std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
{
    std::cout<<"In SearchPositionIK 2"<< std::endl;
    std::cout<< " z: " << ik_pose.position.z <<
                " x: " << ik_pose.position.x <<
                " y: " << ik_pose.position.y <<
                std::endl;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution,
                            IKCallbackFn(), error_code, options);
}
//3
bool TestKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                               double timeout, std::vector<double> &solution,
                                               const IKCallbackFn &solution_callback,
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
{
    std::cout<<"In SearchPositionIK 3"<< std::endl;
    std::cout<< " z: " << ik_pose.position.z <<
                " x: " << ik_pose.position.x <<
                " y: " << ik_pose.position.y <<
                std::endl;
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code, options);
}
//4
bool TestKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                               double timeout, const std::vector<double> &consistency_limits,
                                               std::vector<double> &solution,
                                               const IKCallbackFn &solution_callback,
                                               moveit_msgs::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
{
    std::cout<<"In SearchPositionIK 4"<< std::endl;
    std::cout<< "Commanded Pose: " << " z: " << ik_pose.position.z <<
                                      " x: " << ik_pose.position.x <<
                                      " y: " << ik_pose.position.y <<
                                      std::endl;

    std::cout<<  "IK Seed State: " << " d: " << ik_seed_state[0] <<
                                      " q1: " << ik_seed_state[1] <<
                                      " q2: " << ik_seed_state[2] <<
                                      " q3: " << ik_seed_state[3] <<
                                      std::endl;

    if (!initialized_)
    {
        ROS_ERROR_NAMED("kdl", "kinematics solver not initialized");
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }
    if (ik_seed_state.size() != dimension_)
    {
        ROS_ERROR_STREAM_NAMED("kdl",
                               "Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    std::vector<std::vector<double>> solutions;
    if (!getIKSolutions(ik_pose, solutions))
    {
        ROS_DEBUG_STREAM_NAMED(name_, "No solution whatsoever");
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }
    // find a single solution that is closest to ik_seed_state
    solution = getNearestSolutionToSeed(ik_seed_state, solutions);
    std::cout<<"Solution: " ;
    for (const auto & s : solution)
        std::cout<< s << ", ";
    std::cout<<"." << std::endl;
    if (!solution_callback.empty())
    {
        std::cout<< "SOLUTION CALLBACK PROVIDED"<<std::endl;
        solution_callback(ik_pose, solution, error_code);
        if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_DEBUG_STREAM_NAMED("InHouse", "Solution passes callback");
            return true;
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED("Inhouse", "Solution has error code " << error_code);
            return false;
        }
    }
    else
    {
        error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true; // no collision check callback provided
    }
}

bool TestKinematicsPlugin::isSolutionValid(const double &q) const
{
    // 2.01 bcoz, I dont want to go near the joint_limits
    if ((-M_PI/2.01 <= q) && (q <= M_PI/2.01) )
        return true;
    else
        return false;
}

//PLUGINLIB_EXPORT_CLASS(inhouse_moveit_kinematics_plugin::TestKinematicsPlugin, kinematics::KinematicsBase);