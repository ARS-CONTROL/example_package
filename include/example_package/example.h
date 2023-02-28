#ifndef EXAMPLE_H
#define EXAMPLE_H

#include <signal.h>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>

#include "example_package/example_msg.h"
#include "example_package/example_srv.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Eigen>

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Array<double, 6, 1> Array6d;

struct example_struct {
    double a;
    std::string b;
    std::vector<int> c;
};

#define GET_VARIABLE_NAME(Variable) (#Variable)

#define BOOL_DEFINITION true
#define INT_DEFINITION 10
#define STRING_DEFINITION "String Definition"

class ExamplePackage {

    public:
    
      ExamplePackage(ros::NodeHandle &nh, ros::Rate ros_rate,
                     std::string example_data,
                     std::vector<double> example_vector);

      ~ExamplePackage();

      void spinner(void);

    private:

        // ---- ROS - NODE HANDLE & RATE ---- //
        ros::NodeHandle nh_;
        ros::Rate ros_rate_;

        // ---- GLOBAL VARIABLES ---- //
        bool example_bool_ = false;
        int example_int_ = 100;
        std::string example_string_ = "Default";
        std::vector<double> example_vector_ = {1, 1.1, 0.0};
        Vector6d example_eigen_vector_;
        example_struct example_struct_;

        // ---- ROS - PARAMETERS ---- //
        std::string example_string_param_;
        double example_double_param_;
        bool example_bool_param_;
        std::vector<double> example_vector_param_;

        // ---- ROS - PUBLISHERS ---- //
        ros::Publisher example_publisher_;
        ros::Publisher example_custom_publisher_;

        // ---- ROS - SUBSCRIBERS & CALLBACKS ---- //
        ros::Subscriber example_subscriber_;
        ros::Subscriber example_custom_subscriber_;
        void exampleSubscriberCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void exampleCustomSubscriberCallback(const example_package::example_msg::ConstPtr &msg);

        // ---- ROS - SERVICE CLIENTS ---- //
        ros::ServiceClient example_client_;
        ros::ServiceClient example_custom_client_;

        // ---- ROS - SERVICE SERVERS & CALLBACKS ---- //
        ros::ServiceServer example_server_;
        ros::ServiceServer example_custom_server_;
        bool exampleServerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool exampleCustomServerCallback(example_package::example_srv::Request &req, example_package::example_srv::Response &res);

        // ---- ROS - ACTIONS ---- //
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *action_client_;
        control_msgs::FollowJointTrajectoryGoal trajectory_goal;

        // ---- MoveIt! ROBOT MODEL ---- //
        robot_model_loader::RobotModelLoader robot_model_loader_;
        robot_model::RobotModelPtr kinematic_model_;
        robot_state::RobotStatePtr kinematic_state_;
        const robot_state::JointModelGroup *joint_model_group_;
        std::vector<std::string> joint_names_;
        Eigen::MatrixXd J_;

        // ---- KINEMATIC FUNCTIONS ---- //
        Eigen::Matrix4d computeFK (std::vector<double> joint_position, std::vector<double> joint_velocity);
        Eigen::MatrixXd computeArmJacobian (std::vector<double> joint_position, std::vector<double> joint_velocity);
        Matrix6d getEE_RotationMatrix (std::vector<double> joint_position, std::vector<double> joint_velocity);

        // ---- PUBLISH - CALL FUNCTIONS ---- //
        void PublishMessage (void);
        void CallService (void);
        void CallAction (void);

        // ---- USEFUL FUNCTIONS ---- //
        int getSign(double data);

};

#endif /* EXAMPLE_H */
