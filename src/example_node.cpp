#include "example_package/example.h"

void UR_Stop_Signal_Handler(int) {

    // Start ROS Node
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("ur_stop_node", options);

    // ROS Publisher
    auto joint_group_vel_controller_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    // Create Stop Message
    std_msgs::msg::Float64MultiArray stop_msgs;
    stop_msgs.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
    stop_msgs.layout.dim[0].size = 6;  // Assuming the size is 6
    stop_msgs.layout.dim[0].stride = 1;
    stop_msgs.layout.dim[0].label = "velocity";

    stop_msgs.data.clear();
    stop_msgs.data.resize(6, 0.0);

    joint_group_vel_controller_publisher -> publish(stop_msgs);

    rclcpp::shutdown();

}

void custom_Signal_Handler(int) {

    // DO THINGS ...

    // rclcpp::shutdown();

}

int main(int argc, char **argv) {

    // Initialize ROS
    rclcpp::init(argc, argv);

    // Sig-Int Function (CTRL+C)
    signal(SIGINT, custom_Signal_Handler);
    signal(SIGINT, UR_Stop_Signal_Handler);

    // ROS Initialization Parameters
    rclcpp::Rate ros_rate = rclcpp::Rate(1000);

    // Class Initialization Parameters
    std::string example_string = "example_data";
    std::vector<double> example_vector = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};

    ExamplePackage *ep = new ExamplePackage(example_string, example_vector);

    while (rclcpp::ok())
    {

        ep -> spinner();

    }

    delete ep;

return 0;

}
