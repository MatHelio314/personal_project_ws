#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class BalanceController : public rclcpp::Node
{
public:
    BalanceController() : Node("balance_controller"), kp_(1.0), ki_(0.0), kd_(0.0), prev_error_(0.0), integral_(0.0)
    {
        // Subscriber to IMU data
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&BalanceController::imu_callback, this, std::placeholders::_1));
        
        // Publisher to cmd_vel
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/diff_controller/cmd_vel", 10);
        
        // PID parameters (can be adjusted)
        this->declare_parameter("kp", 2.55);
        this->declare_parameter("ki", 5.0);
        this->declare_parameter("kd", 0.1);

        // Get PID parameters from the parameter server
        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);

        // Initialize time for derivative calculation
        last_time_ = this->now();
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double y_orientation = msg->orientation.y;
        double target_y = 0.0; // Target y orientation is 0 (robot upright)
        
        // Calculate error
        double error = target_y - y_orientation;

        // Proportional term
        double p_term = kp_ * error;

        // Integral term
        integral_ += error;
        double i_term = ki_ * integral_;

        // Derivative term
        rclcpp::Time current_time = this->now();
        double delta_time = (current_time - last_time_).seconds();
        double derivative = (error - prev_error_) / delta_time;
        double d_term = kd_ * derivative;

        // Calculate control effort (PID output)
        double control_effort = p_term + i_term + d_term;

        // Create and publish velocity command
        geometry_msgs::msg::TwistStamped cmd_msg;
        cmd_msg.twist.linear.x = -control_effort; // Move forward/backward based on control effort
        cmd_msg.twist.angular.z = 0.0; // No rotation
        
        cmd_vel_publisher_->publish(cmd_msg);

        // Update for next iteration
        prev_error_ = error;
        last_time_ = current_time;
    }

    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

    // PID variables
    double kp_, ki_, kd_;
    double prev_error_, integral_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalanceController>());
    rclcpp::shutdown();
    return 0;
}






// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"

// class BalanceController : public rclcpp::Node
// {
// public:
//     BalanceController() : Node("balance_controller")
//     {
//         imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/imu/data", 10, std::bind(&BalanceController::imuCallback, this, std::placeholders::_1));
//         cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
//             "/diff_drive_controller/cmd_vel", 10);

//         // PID gains (tune these based on your system)
//         Kp_ = 1.0;
//         Ki_ = 0.0;
//         Kd_ = 0.0;

//         prev_error_ = 0.0;
//         integral_ = 0.0;
//         last_time_ = this->now();
//     }

// private:
//     void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
//     {
//         // Extract the y-orientation from the IMU data
//         double orientation_y = msg->orientation.y;

//         // Calculate the error (goal is to keep orientation close to 0)
//         double error = 0.0 - orientation_y;

//         // Time delta
//         auto current_time = this->now();
//         double dt = (current_time - last_time_).seconds();

//         // Proportional term
//         double Pout = Kp_ * error;

//         // Integral term
//         integral_ += error * dt;
//         double Iout = Ki_ * integral_;

//         // Derivative term
//         double derivative = (error - prev_error_) / dt;
//         double Dout = Kd_ * derivative;

//         // PID output
//         double output = Pout + Iout + Dout;

//         // Publish to /cmd_vel
//         geometry_msgs::msg::TwistStamped cmd_msg;
//         cmd_msg.twist.linear.x = output;  // control the forward/backward movement
//         cmd_msg.twist.linear.y = 0.0;
//         cmd_msg.twist.linear.z = 0.0;
//         cmd_msg.twist.angular.x = 0.0;
//         cmd_msg.twist.angular.y = 0.0;
//         cmd_msg.twist.angular.z = 0.0;

//         cmd_vel_publisher_->publish(cmd_msg);

//         // Update for the next iteration
//         prev_error_ = error;
//         last_time_ = current_time;
//     }

//     // Subscribers and Publishers
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
//     rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

//     // PID variables
//     double Kp_, Ki_, Kd_;
//     double prev_error_, integral_;
//     rclcpp::Time last_time_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<BalanceController>());
//     rclcpp::shutdown();
//     return 0;
// }






// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <geometry_msgs/msg/twist_stamped.hpp>

// class BalanceController : public rclcpp::Node
// {
// public:
//     BalanceController() : Node("balance_controller"), kp_balance_(1.0), ki_balance_(0.0), kd_balance_(0.0),
//                           kp_velocity_(1.0), ki_velocity_(0.0), kd_velocity_(0.0),
//                           prev_error_balance_(0.0), integral_balance_(0.0), 
//                           prev_error_velocity_(0.0), integral_velocity_(0.0)
//     {
//         // Subscriber to IMU data
//         imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/imu/data", 10, std::bind(&BalanceController::imu_callback, this, std::placeholders::_1));

//         vel_subscriber_ = this->create_subscription<control_msgs::msg::DynamicJointState>(
//             "dynamic_joint_states", 10, ......FINISH THIS.....
//         )
        
//         // Publisher to cmd_vel
//         cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
//             "/diff_drive_controller/cmd_vel", 10);
        
//         // PID parameters for balance
//         this->declare_parameter("kp_balance", 2.55);
//         this->declare_parameter("ki_balance", 5.0);
//         this->declare_parameter("kd_balance", 0.1);

//         // PID parameters for velocity
//         this->declare_parameter("kp_velocity", 0.1);
//         this->declare_parameter("ki_velocity", 0.0);
//         this->declare_parameter("kd_velocity", 0.0);

//         // Get PID parameters from the parameter server
//         this->get_parameter("kp_balance", kp_balance_);
//         this->get_parameter("ki_balance", ki_balance_);
//         this->get_parameter("kd_balance", kd_balance_);
//         this->get_parameter("kp_velocity", kp_velocity_);
//         this->get_parameter("ki_velocity", ki_velocity_);
//         this->get_parameter("kd_velocity", kd_velocity_);

//         // Initialize time for derivative calculation
//         last_time_ = this->now();
//     }

// private:
//     void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
//     {
//         // Extract the y orientation value (balancing)
//         double y_orientation = msg->orientation.y;
//         double target_y = 0.0; // Target y orientation is 0 (robot upright)
        
//         // Balance PID
//         double error_balance = target_y - y_orientation;
//         double p_term_balance = kp_balance_ * error_balance;
//         integral_balance_ += error_balance;
//         double i_term_balance = ki_balance_ * integral_balance_;
        
//         rclcpp::Time current_time = this->now();
//         double delta_time = (current_time - last_time_).seconds();
//         double derivative_balance = (error_balance - prev_error_balance_) / delta_time;
//         double d_term_balance = kd_balance_ * derivative_balance;
        
//         double control_effort_balance = p_term_balance + i_term_balance + d_term_balance;

//         // Get the current linear velocity (to regulate it to zero)
//         double current_velocity = control_effort_balance; // Assume the velocity is proportional to control effort
        
        
//         // Velocity PID
//         double target_velocity = 0.0; // We want the velocity to be zero
//         double error_velocity = target_velocity - current_velocity;
        
//         double p_term_velocity = kp_velocity_ * error_velocity;
//         integral_velocity_ += error_velocity;
//         double i_term_velocity = ki_velocity_ * integral_velocity_;
//         double derivative_velocity = (error_velocity - prev_error_velocity_) / delta_time;
//         double d_term_velocity = kd_velocity_ * derivative_velocity;

//         double control_effort_velocity = p_term_velocity + i_term_velocity + d_term_velocity;

//         // Combine the balance PID and velocity PID
//         double combined_control_effort = (- control_effort_balance) - control_effort_velocity;

//         // Create and publish velocity command
//         geometry_msgs::msg::TwistStamped cmd_msg;
//         cmd_msg.twist.linear.x = combined_control_effort; // Move forward/backward based on control effort
//         cmd_msg.twist.angular.z = 0.0; // No rotation

//         cmd_vel_publisher_->publish(cmd_msg);

//         // Update previous errors and time
//         prev_error_balance_ = error_balance;
//         prev_error_velocity_ = error_velocity;
//         last_time_ = current_time;
//     }

//     // Subscribers and publishers
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
//     rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

//     // PID variables for balance control
//     double kp_balance_, ki_balance_, kd_balance_;
//     double prev_error_balance_, integral_balance_;

//     // PID variables for velocity control
//     double kp_velocity_, ki_velocity_, kd_velocity_;
//     double prev_error_velocity_, integral_velocity_;

//     // Time tracking
//     rclcpp::Time last_time_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<BalanceController>());
//     rclcpp::shutdown();
//     return 0;
// }
