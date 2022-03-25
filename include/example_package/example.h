#ifndef EXAMPLE_H
#define EXAMPLE_H

#include <signal.h>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
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

class ExamplePackage {

    public:
    
      ExamplePackage(ros::NodeHandle &n, ros::Rate ros_rate,
                     std::string example_data,
                     std::vector<double> example_vector);

      ~ExamplePackage();

      void spinner(void);

      bool debug_;
      bool offstream_debug_;

    private:

        // ---- ROS - NODE HANDLE & RATE ---- //
        ros::NodeHandle nh_;
        ros::Rate loop_rate_;

        // ---- GLOBAL VARIABLES ---- //
        example_struct a_;
        std::vector<double> b_;
        Vector6d c_;

        // ---- MoveIt! ROBOT MODEL ---- //
        robot_model_loader::RobotModelLoader robot_model_loader_;
        robot_model::RobotModelPtr kinematic_model_;
        robot_state::RobotStatePtr kinematic_state_;
        const robot_state::JointModelGroup *joint_model_group_;
        std::vector<std::string> joint_names_;
        Eigen::MatrixXd J_;

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

        // ---- ROS ACTIONS ---- //
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_client;
        control_msgs::FollowJointTrajectoryGoal trajectory_goal;

        // ---- USEFUL FILTERS ---- //
        std::vector<double> Low_Pass_Filter(std::vector<double> data, int filter_dimensions = 100);

        // ---- USEFUL FUNCTIONS ---- //
        int getSign(double data);

        // ---- OFFSTREAM DEBUG ---- //
        std::ofstream ofstream_debug_;

};

#endif /* EXAMPLE_H */
