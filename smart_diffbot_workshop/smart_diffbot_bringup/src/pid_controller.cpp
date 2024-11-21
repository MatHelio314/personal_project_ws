#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  // Use Float64MultiArray

class PidControllerNode : public rclcpp::Node
{
public:
    PidControllerNode() : Node("pid_controllers")
    {
        // Subscriber to IMU data on /imu/data
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&PidControllerNode::imu_callback, this, std::placeholders::_1)
        );

        // Publisher to position commands on /position_controller/commands
        position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/position_controller/commands", 10
        );
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract the y orientation from the IMU data
        double y_orientation = msg->orientation.y;

        // Create a position command message with Float64MultiArray
        std_msgs::msg::Float64MultiArray command_msg;
        
        // Fill the data with relevant control values
        command_msg.data.push_back(y_orientation); // Example: Add y_orientation
        command_msg.data.push_back(y_orientation); // Placeholder for additional values if needed

        // Publish the command to /position_controller/commands
        position_publisher_->publish(command_msg);

        RCLCPP_INFO(this->get_logger(), "Published command: [%f, %f]", command_msg.data[0], command_msg.data[1]);
    }

    // Subscriber and Publisher
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidControllerNode>());
    rclcpp::shutdown();
    return 0;
}
