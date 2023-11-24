#include "example_package/example.h"

//----------------------------------------------------- CONSTRUCTOR -----------------------------------------------------//

ExamplePackage::ExamplePackage(std::string example_data, std::vector<double> example_vector):
    Node ("example_node"), example_string_(example_data), example_vector_(example_vector)
{

    // ---- LOAD GLOBAL PARAMETERS ---- //
    if (!get_parameter_or("/example_string_param", example_string_param_, std::string("Default Example"))) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"example_string_param parameter\" | Using Default: " << example_string_param_);

    if (!get_parameter_or("/example_double_param", example_double_param_, 1.1)) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"example_double_param parameter\" | Using Default: " << example_double_param_);

    // ---- LOAD NODE SPECIFIC PARAMETERS ---- //
     if (!get_parameter_or("/example_cpp_node/example_bool_param", example_bool_param_, true)) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"example_bool_param parameter\" | Using Default: " << example_bool_param_);

    if (!get_parameter_or("/example_cpp_node/example_vector_param", example_vector_param_, {1.1, 1.1})) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"example_vector_param parameter\" | Using Default: " << example_vector_param_[0] << ", " << example_vector_param_[1]);

    // ---- LOAD NAMESPACED PARAMETERS ---- //
    if (!get_parameter_or("/namespace_1/example_string_param", example_string_param_, std::string("Default Example"))) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"example_string_param parameter\" | Using Default: " << example_string_param_);

    if (!get_parameter_or("/namespace_1/example_double_param", example_double_param_, 1.1)) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"example_double_param parameter\" | Using Default: " << example_double_param_);

    // ---- LOAD YAML FILE PARAMETERS ---- //
    if (!get_parameter_or("/yaml_string_param", example_string_param_, std::string("Default Example"))) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"yaml_string_param parameter\" | Using Default: " << example_string_param_);

    if (!get_parameter_or("/yaml_double_param", example_double_param_, 1.1)) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"yaml_double_param parameter\" | Using Default: " << example_double_param_);

     if (!get_parameter_or("/yaml_bool_param", example_bool_param_, true)) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"yaml_bool_param parameter\" | Using Default: " << example_bool_param_);

    if (!get_parameter_or("/yaml_vector_param", example_vector_param_, {1.1, 1.1})) 
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to read the \"yaml_vector_param parameter\" | Using Default: " << example_vector_param_[0] << ", " << example_vector_param_[1]);

    // ---- ROS - PUBLISHERS ---- //
    string_publisher_          = create_publisher<std_msgs::msg::String>("/string_topic_name", 1);
    example_publisher_         = create_publisher<trajectory_msgs::msg::JointTrajectory>("/global_example_publisher_topic_name", 1);
    example_custom_publisher_  = create_publisher<example_package::msg::ExampleMsg>("local_example_publisher_topic_name", 1);

    // ---- ROS - SUBSCRIBERS ---- //
    string_subscriber_         = create_subscription<std_msgs::msg::String>("string_topic_name", 1, std::bind(&ExamplePackage::stringSubscriberCallback, this, std::placeholders::_1));
    example_subscriber_        = create_subscription<std_msgs::msg::Float64MultiArray>("/global_example_subscriber_topic_name", 1, std::bind(&ExamplePackage::exampleSubscriberCallback, this, std::placeholders::_1));
    example_custom_subscriber_ = create_subscription<example_package::msg::ExampleMsg>("/group/example_subscriber_topic_name",  1, std::bind(&ExamplePackage::exampleCustomSubscriberCallback, this, std::placeholders::_1));

    // ---- ROS - SERVICE CLIENTS ---- //
    example_client_            = create_client<std_srvs::srv::SetBool>("/global_example_server_name");
    example_custom_client_     = create_client<example_package::srv::ExampleSrv>("/group/example_server_name");

    // ---- ROS - SERVICE SERVERS ---- //
    example_server_            = create_service<std_srvs::srv::SetBool>("/global_example_server_name", std::bind(&ExamplePackage::exampleServerCallback, this, std::placeholders::_1, std::placeholders::_2));
    example_custom_server_     = create_service<example_package::srv::ExampleSrv>("/group/example_server_name", std::bind(&ExamplePackage::exampleCustomServerCallback, this, std::placeholders::_1, std::placeholders::_2));

    // ---- ROS - ACTIONS ---- //
    trajectory_action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this, "/trajectory_publisher_action_name");

    // ---- DEBUG PRINT ---- //
    RCLCPP_INFO_STREAM(get_logger(), "Info Print");
    RCLCPP_WARN_STREAM(get_logger(), "Warn Print");
    RCLCPP_ERROR_STREAM(get_logger(), "Error Print");
    RCLCPP_INFO_ONCE(get_logger(), "Printf Print Once: %.2f", 10.0);
    RCLCPP_INFO_STREAM_ONCE(get_logger(), "cout Print Once: " << 10 << std::endl);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Print Every n Milliseconds");
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000, "Print Every n Milliseconds, Ignoring the First One");

}

//----------------------------------------------------- DESTRUCTOR ------------------------------------------------------//

ExamplePackage::~ExamplePackage() {}

//------------------------------------------------------- MOVEIT --------------------------------------------------------//

void ExamplePackage::initializeRobotModelLoader(std::shared_ptr<ExamplePackage> node) {

    // ---- MoveIt Robot Model ---- //
    robot_model_loader_ = new robot_model_loader::RobotModelLoader(node, "robot_description");
    kinematic_model_    = robot_model_loader_ -> getModel();
    kinematic_state_    = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model_));
    kinematic_state_    -> setToDefaultValues();
    joint_model_group_  = kinematic_model_ -> getJointModelGroup("manipulator");
    joint_names_        = joint_model_group_ -> getJointModelNames();

}

//------------------------------------------------ SUBSCRIBER CALLBACKS -------------------------------------------------//

void ExamplePackage::stringSubscriberCallback(const std_msgs::msg::String::SharedPtr msg) {

    std_msgs::msg::String received_msg = *msg;
    RCLCPP_INFO_STREAM(get_logger(), "Received String Message: " << received_msg.data);

    // DO THINGS...

}

void ExamplePackage::exampleSubscriberCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

    std_msgs::msg::Float64MultiArray received_msg = *msg;

    // DO THINGS...

}

void ExamplePackage::exampleCustomSubscriberCallback(const example_package::msg::ExampleMsg::SharedPtr msg) {

    example_package::msg::ExampleMsg received_custom_msg = *msg;

    // DO THINGS...

}

//-------------------------------------------------- SERVER CALLBACKS ---------------------------------------------------//

bool ExamplePackage::exampleServerCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

    bool received_request = request->data;
    RCLCPP_INFO_STREAM(get_logger(), "Received Service Request with Value: " << received_request);

    // DO THINGS...

    // Fill the Response
    response->success = true;

    // Return true to Indicate a Successful Service Response
    return true;

}

bool ExamplePackage::exampleCustomServerCallback(const std::shared_ptr<example_package::srv::ExampleSrv::Request> request, std::shared_ptr<example_package::srv::ExampleSrv::Response> response) {

    example_package::srv::ExampleSrv::Request received_request = *request;

    std::string string_value = received_request.string_value;
    std_msgs::msg::Float32 std_msgs_float = received_request.std_msgs_float;
    // ...

    // DO THINGS...

    response->int_value = std_msgs_float.data;
    //...

    response->success = true;
    return true;

}

//------------------------------------------------- KINEMATIC FUNCTIONS -------------------------------------------------//

Eigen::Matrix4d ExamplePackage::computeFK(std::vector<double> joint_position, std::vector<double> joint_velocity) {

    /* Compute Forward Kinematic */

    rclcpp::spin_some(this->get_node_base_interface());

    // //Update MoveIt! Kinematic Model
    // kinematic_state_ -> setJointGroupPositions  (joint_model_group_, joint_position);
    // kinematic_state_ -> setJointGroupVelocities (joint_model_group_, joint_velocity);
    // kinematic_state_ -> enforceBounds();

    // // Computing the actual position of the end-effector using Forward Kinematic respect "world"
    // const Eigen::Affine3d& end_effector_state = kinematic_state_ -> getGlobalLinkTransform("tool0");

    // // Get the Translation Vector and Rotation Matrix
    // Eigen::Vector3d translation_vector = end_effector_state.translation();
    // Eigen::Matrix3d rotation_matrix    = end_effector_state.rotation();

    // //Transformation Matrix
    // Eigen::Matrix4d transformation_matrix;
    // transformation_matrix.setZero();

    // //Set Identity to make bottom row of Matrix 0,0,0,1
    // transformation_matrix.setIdentity();

    // transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    // transformation_matrix.block<3,1>(0,3) = translation_vector;

    // return transformation_matrix;
    auto a = joint_position;
    auto b = joint_velocity;
    return Eigen::Matrix4d::Identity();

}

Eigen::MatrixXd ExamplePackage::computeArmJacobian(std::vector<double> joint_position, std::vector<double> joint_velocity) {

    /* Compute Manipulator Jacobian */

    // ros::spinOnce();

    // //Update MoveIt! Kinematic Model
    // kinematic_state_ -> setJointGroupPositions  (joint_model_group_, joint_position);
    // kinematic_state_ -> setJointGroupVelocities (joint_model_group_, joint_velocity);
    // kinematic_state_ -> enforceBounds();

    // // Computing the Jacobian of the arm
    // Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    // Eigen::MatrixXd jacobian;

    // kinematic_state_ -> getJacobian(joint_model_group_, kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()), reference_point_position, jacobian);

    // return jacobian;
    auto a = joint_position;
    auto b = joint_velocity;
    return Eigen::MatrixXd::Identity(6,6);
}

Matrix6d ExamplePackage::getEE_RotationMatrix(std::vector<double> joint_position, std::vector<double> joint_velocity) {

    /* Get End-Effector Rotation Matrix */

    // ros::spinOnce();

    // //Update MoveIt! Kinematic Model
    // kinematic_state_ -> setJointGroupPositions  (joint_model_group_, joint_position);
    // kinematic_state_ -> setJointGroupVelocities (joint_model_group_, joint_velocity);
    // kinematic_state_ -> enforceBounds();

    // // Computing the actual position of the end-effector using Forward Kinematic respect "world"
    // const Eigen::Affine3d& end_effector_state = kinematic_state_ -> getGlobalLinkTransform("tool0");

    // // Rotation Matrix 6x6
    // Matrix6d rotation_matrix;
    // rotation_matrix.setZero();

    // rotation_matrix.topLeftCorner(3, 3)     = end_effector_state.rotation();
    // rotation_matrix.bottomRightCorner(3, 3) = end_effector_state.rotation();

    // Eigen::Vector3d euler_angles = end_effector_state.rotation().eulerAngles(0, 1, 2);
    // Eigen::Quaterniond rotation_quaternion(end_effector_state.rotation());

    // return rotation_matrix;
    auto a = joint_position;
    auto b = joint_velocity;
    return Matrix6d::Identity();
}

//----------------------------------------------- PUBLISH - CALL FUNCTIONS ----------------------------------------------//

void ExamplePackage::PublishMessage(void) {

    // ROS Message Creation
    trajectory_msgs::msg::JointTrajectory trajectory_temp;

    // ROS Message Filling
    trajectory_temp.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    trajectory_temp.points.resize(1);
    trajectory_temp.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    trajectory_temp.points[0].time_from_start = rclcpp::Duration(5,0);

    // ROS Message Publication Publish Trajectory Position
    example_publisher_ -> publish(trajectory_temp);

}

void ExamplePackage::CallService(void) {

    // Wait for Service to Start
    if (!example_client_ -> wait_for_service(1s)) {RCLCPP_ERROR(get_logger(), "Service \"/global_example_server_name\" Not Available");}

    // Service Request Creation
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request -> data = true;

    // Call Service
    auto result_future = example_client_ -> async_send_request(request);

    // Wait for the Result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {RCLCPP_ERROR_STREAM(get_logger(), "Failed to Call Service: \"/global_example_server_name\"");}

    // Get and Print the Result
    auto result = result_future.get();
    RCLCPP_INFO_STREAM(get_logger(), "Service response: " << result->success);

}

void ExamplePackage::CallAction(void) {

    // Wait for Service to Start
    if (!trajectory_action_client_ -> wait_for_action_server()) {RCLCPP_ERROR(get_logger(), "Action Service \"/trajectory_publisher_action_name\" Not Available");}

    // Action Goal Filling
    auto goal = control_msgs::action::FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    goal.trajectory.points[0].time_from_start = rclcpp::Duration(5,0);

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&ExamplePackage::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback      = std::bind(&ExamplePackage::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback        = std::bind(&ExamplePackage::resultCallback, this, std::placeholders::_1);

    auto future_goal_handle = trajectory_action_client_ -> async_send_goal(goal, send_goal_options);

}

void ExamplePackage::goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr> future) {

    // Get the Goal Handle
    auto goal_handle = future.get();

    if (!goal_handle) {RCLCPP_ERROR(get_logger(), "Goal was rejected by server");}
    else {RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");}

}

void ExamplePackage::feedbackCallback(rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback) {

    // Get the Feedback
    RCLCPP_INFO_STREAM(get_logger(), "feedback: " << feedback->joint_names[0] << " = " << feedback->actual.positions[0]);

}

void ExamplePackage::resultCallback(const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::WrappedResult & result) {

    switch (result.code) {

        // Goal was Accepted, Aborted, or Cancelled
        case rclcpp_action::ResultCode::SUCCEEDED: break;
        case rclcpp_action::ResultCode::ABORTED:   RCLCPP_ERROR(get_logger(), "Goal Aborted");  return;
        case rclcpp_action::ResultCode::CANCELED:  RCLCPP_ERROR(get_logger(), "Goal Canceled"); return;
        default: RCLCPP_ERROR(get_logger(), "Unknown Result Code"); return;
    }

    // Print the Result
    RCLCPP_INFO_STREAM(get_logger(), "Result received: " << result.result->error_code);

}

//--------------------------------------------------- UTILS FUNCTIONS ---------------------------------------------------//

int ExamplePackage::getSign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}

//-------------------------------------------------------- MAIN --------------------------------------------------------//

void ExamplePackage::spinner (void) {

    // Spin Once;
    rclcpp::spin_some(this->get_node_base_interface());

    // Publish a ROS Message
    PublishMessage();

    // Call a ROS Service Client
    CallService();

    // Call a ROS Action Client
    CallAction();

}
