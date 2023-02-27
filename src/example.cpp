#include "example_package/example.h"

//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//
    
ExamplePackage::ExamplePackage(
    ros::NodeHandle &nh, ros::Rate ros_rate, std::string example_data, std::vector<double> example_vector):
    nh_(nh), ros_rate_(ros_rate), example_string_(example_data), example_vector_(example_vector)
{
    
    // ---- LOAD GLOBAL PARAMETERS ---- //
    if (!nh.param<std::string>("/example_string_param", example_string_param_, "Default Example")) 
        ROS_ERROR_STREAM("Failed to read the \"example_string_param parameter\" | Using Default: " << example_string_param_);
    
    if (!nh.param<double>("/example_double_param", example_double_param_, 1.1)) 
        ROS_ERROR_STREAM("Failed to read the \"example_double_param parameter\" | Using Default: " << example_double_param_);

    // ---- LOAD NODE SPECIFIC PARAMETERS ---- //
     if (!nh.param<bool>("/example_cpp_node/example_bool_param", example_bool_param_, true)) 
        ROS_ERROR_STREAM("Failed to read the \"example_bool_param parameter\" | Using Default: " << example_bool_param_);

    if (!nh.param<std::vector<double>>("/example_cpp_node/example_vector_param", example_vector_param_, {1.1, 1.1})) 
        ROS_ERROR_STREAM("Failed to read the \"example_vector_param parameter\" | Using Default: " << example_vector_param_[0] << ", " << example_vector_param_[1]);

    // ---- LOAD NAMESPACED PARAMETERS ---- //
    if (!nh.param<std::string>("/namespace_1/example_string_param", example_string_param_, "Default Example")) 
        ROS_ERROR_STREAM("Failed to read the \"example_string_param parameter\" | Using Default: " << example_string_param_);
    
    if (!nh.param<double>("/namespace_1/example_double_param", example_double_param_, 1.1)) 
        ROS_ERROR_STREAM("Failed to read the \"example_double_param parameter\" | Using Default: " << example_double_param_);

     // ---- LOAD YAML FILE PARAMETERS ---- //
    if (!nh.param<std::string>("/yaml_string_param", example_string_param_, "Default Example")) 
        ROS_ERROR_STREAM("Failed to read the \"yaml_string_param parameter\" | Using Default: " << example_string_param_);
    
    if (!nh.param<double>("/yaml_double_param", example_double_param_, 1.1)) 
        ROS_ERROR_STREAM("Failed to read the \"yaml_double_param parameter\" | Using Default: " << example_double_param_);

     if (!nh.param<bool>("/yaml_bool_param", example_bool_param_, true)) 
        ROS_ERROR_STREAM("Failed to read the \"yaml_bool_param parameter\" | Using Default: " << example_bool_param_);

    if (!nh.param<std::vector<double>>("/yaml_vector_param", example_vector_param_, {1.1, 1.1})) 
        ROS_ERROR_STREAM("Failed to read the \"yaml_vector_param parameter\" | Using Default: " << example_vector_param_[0] << ", " << example_vector_param_[1]);


    // ---- ROS - PUBLISHERS ---- //
    example_publisher_         = nh.advertise<trajectory_msgs::JointTrajectory>("/global_example_publisher_topic_name", 1);
    example_custom_publisher_  = nh.advertise<example_package::example_msg>("local_example_publisher_topic_name", 1);

    // ---- ROS - SUBSCRIBERS ---- //
    example_subscriber_        = nh.subscribe("/global_example_subscriber_topic_name", 1, &ExamplePackage::exampleSubscriberCallback, this);
    example_custom_subscriber_ = nh.subscribe("/group/example_subscriber_topic_name", 1, &ExamplePackage::exampleCustomSubscriberCallback, this);
    
    // ---- ROS - SERVICE SERVERS ---- //
    example_server_            = nh.advertiseService("/global_example_server_name", &ExamplePackage::exampleServerCallback, this);
    example_custom_server_     = nh.advertiseService("/group/example_server_name", &ExamplePackage::exampleCustomServerCallback, this);

    // ---- ROS - SERVICE CLIENTS ---- //
    example_client_            = nh.serviceClient<std_srvs::SetBool>("/global_example_server_name");
    example_custom_client_     = nh.serviceClient<example_package::example_srv>("/group/example_server_name");

    // ---- ROS - ACTIONS ---- //
    trajectory_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/trajectory_publisher_action_name", true);

    // ---- MoveIt Robot Model ---- //
    robot_model_loader_ = robot_model_loader::RobotModelLoader ("robot_description");
    kinematic_model_    = robot_model_loader_.getModel();
    kinematic_state_    = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
    kinematic_state_    -> setToDefaultValues();
    joint_model_group_  = kinematic_model_   -> getJointModelGroup("manipulator");
    joint_names_        = joint_model_group_ -> getJointModelNames();

    // ---- DEBUG PRINT ---- //
    ROS_INFO("Info Print");
    ROS_WARN("Warn Print");
    ROS_ERROR("Error Print");
    ROS_INFO_STREAM_ONCE("cout Print Once: " << 10 << std::endl);
    ROS_INFO_ONCE("Printf Print Once: %.2f", 10);
    ROS_INFO_THROTTLE(10, "Print Every n Seconds");
    ROS_INFO_DELAYED_THROTTLE(10, "Print Every n Seconds, Ignoring the First One");

}

//----------------------------------------------------- DESTRUCTOR ------------------------------------------------------//

ExamplePackage::~ExamplePackage() {}

//------------------------------------------------ SUBSCRIBER CALLBACKS -------------------------------------------------//

void ExamplePackage::exampleSubscriberCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {

    std_msgs::Float64MultiArray received_msg = *msg;

    // DO THINGS...

}

void ExamplePackage::exampleCustomSubscriberCallback(const example_package::example_msg::ConstPtr &msg) {

    example_package::example_msg received_custom_msg = *msg;

    // DO THINGS...

}

//-------------------------------------------------- SERVER CALLBACKS ---------------------------------------------------//

bool ExamplePackage::exampleServerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {

    bool received_request = req.data;

    // DO THINGS...

    res.success = true;
    return true;

}

bool ExamplePackage::exampleCustomServerCallback(example_package::example_srv::Request &req, example_package::example_srv::Response &res) {

    example_package::example_srv::Request received_request = req;

    std::string string_value = received_request.string_value;
    std_msgs::Float32 std_msgs_float = received_request.std_msgs_float;
    // ...

    // DO THINGS...

    res.int_value = 23;
    //...

    res.success = true;
    return true;

}

//------------------------------------------------- KINEMATIC FUNCTIONS -------------------------------------------------//

Eigen::Matrix4d ExamplePackage::computeFK (std::vector<double> joint_position, std::vector<double> joint_velocity) {

    /* Compute Forward Kinematic */

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state_ -> setJointGroupPositions  (joint_model_group_, joint_position);
    kinematic_state_ -> setJointGroupVelocities (joint_model_group_, joint_velocity);
    kinematic_state_ -> enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "world"
    const Eigen::Affine3d& end_effector_state = kinematic_state_ -> getGlobalLinkTransform("tool0");

    // Get the Translation Vector and Rotation Matrix
    Eigen::Vector3d translation_vector = end_effector_state.translation();
    Eigen::Matrix3d rotation_matrix    = end_effector_state.rotation();

    //Transformation Matrix
    Eigen::Matrix4d transformation_matrix;
    transformation_matrix.setZero();

    //Set Identity to make bottom row of Matrix 0,0,0,1
    transformation_matrix.setIdentity();

    transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    transformation_matrix.block<3,1>(0,3) = translation_vector;

    return transformation_matrix;

}

Eigen::MatrixXd ExamplePackage::computeArmJacobian (std::vector<double> joint_position, std::vector<double> joint_velocity) {

    /* Compute Manipulator Jacobian */

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state_ -> setJointGroupPositions  (joint_model_group_, joint_position);
    kinematic_state_ -> setJointGroupVelocities (joint_model_group_, joint_velocity);
    kinematic_state_ -> enforceBounds();

    // Computing the Jacobian of the arm
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;

    kinematic_state_ -> getJacobian(joint_model_group_, kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()), reference_point_position, jacobian);

    return jacobian;

}

Matrix6d ExamplePackage::getEE_RotationMatrix (std::vector<double> joint_position, std::vector<double> joint_velocity) {

    /* Get End-Effector Rotation Matrix */

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state_ -> setJointGroupPositions  (joint_model_group_, joint_position);
    kinematic_state_ -> setJointGroupVelocities (joint_model_group_, joint_velocity);
    kinematic_state_ -> enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "world"
    const Eigen::Affine3d& end_effector_state = kinematic_state_ -> getGlobalLinkTransform("tool0");

    // Rotation Matrix 6x6
    Matrix6d rotation_matrix;
    rotation_matrix.setZero();

    rotation_matrix.topLeftCorner(3, 3)     = end_effector_state.rotation();
    rotation_matrix.bottomRightCorner(3, 3) = end_effector_state.rotation();

    Eigen::Vector3d euler_angles = end_effector_state.rotation().eulerAngles(0, 1, 2);
    Eigen::Quaterniond rotation_quaternion(end_effector_state.rotation());

    return rotation_matrix;

}

//----------------------------------------------- PUBLISH - CALL FUNCTIONS ----------------------------------------------//

void ExamplePackage::PublishMessage (void) {

    // ROS Message Creation
    trajectory_msgs::JointTrajectory trajectory_temp;

    // ROS Message Filling
    trajectory_temp.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    trajectory_temp.points.resize(1);
    trajectory_temp.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    trajectory_temp.points[0].time_from_start = ros::Duration(5);

    // ROS Message Publication Publish Trajectory Position
    example_publisher_.publish(trajectory_temp);

}

void ExamplePackage::CallService (void) {

    // Service Creation
    std_srvs::SetBool example_srv;
    
    // Service Filling
    example_srv.request.data = true;

    // Call Service
    if (example_client_.call(example_srv)) {}
    else {ROS_ERROR("Failed to Call Service: \"/global_example_server_name\"");}
    
}


//--------------------------------------------------- UTILS FUNCTIONS ---------------------------------------------------//

int ExamplePackage::getSign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}

//-------------------------------------------------------- MAIN --------------------------------------------------------//

void ExamplePackage::spinner (void) {

    ros::spinOnce();

    // Publish a ROS Message
    PublishMessage();

    // Call a ROS Server/Client
    CallService();

}
