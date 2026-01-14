#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <stdlib.h>
#include <cmath>

using namespace std;

const double PI = 3.1415926;

class TwistToAckermann : public rclcpp::Node {
public:
    TwistToAckermann() : Node("twist_to_ackermann") {
        // Declare parameters
        wheelbase_ = this->declare_parameter<double>("wheelbase", 0.3);  // Distance between front and rear axles (meters)
        max_steering_angle_ = this->declare_parameter<double>("max_steering_angle", 30 * PI/180);  // ~30 degrees (rad)
        
        // Create subscriber
        twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 
            10,
            std::bind(&TwistToAckermann::twistCallback, this, std::placeholders::_1)
        );
        
        // Create publisher
        ackermann_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/ackermann_cmd", 
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "Twist to Ackermann converter initialized");
    }
    
private:
    void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg) {
        double linear_vel = twist_msg->twist.linear.x;
        double angular_vel = twist_msg->twist.angular.z;
        
        // Apply Ackermann constraints
        double vehicleSpeed = linear_vel; 
        double vehicleYawRate = angular_vel;
        double steeringAngle = 0;
        
        if (fabs(vehicleYawRate) > 0.01) {
        // Scale speed based on how much turning is needed
        double desiredSteeringAngle = atan(vehicleYawRate * wheelbase_ / std::max(fabs(vehicleSpeed), 0.01));
        double steeringDemand = std::min(fabs(desiredSteeringAngle) / max_steering_angle_, 1.0);
        
        // More aggressive speed for sharper turns
        double minTurnSpeed = 0.15 + steeringDemand * 0.35;  // 0.15-0.5 m/s based on steering
        
        if (fabs(vehicleSpeed) < minTurnSpeed) {
            vehicleSpeed = (vehicleSpeed >= 0) ? minTurnSpeed : -minTurnSpeed;
        }
        
        steeringAngle = atan(vehicleYawRate * wheelbase_ / vehicleSpeed);
        steeringAngle = std::clamp(steeringAngle, -max_steering_angle_, max_steering_angle_);
        
        vehicleYawRate = (vehicleSpeed / wheelbase_) * tan(steeringAngle);
        }

        auto ackermann_msg = geometry_msgs::msg::TwistStamped();
        ackermann_msg.header.stamp = this->get_clock()->now();
        ackermann_msg.header.frame_id = "base_link";
        ackermann_msg.twist.linear.x = vehicleSpeed;
        ackermann_msg.twist.angular.z = vehicleYawRate;
        ackermann_pub_->publish(ackermann_msg);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ackermann_pub_;
    
    double wheelbase_;
    double max_steering_angle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToAckermann>());
    rclcpp::shutdown();
    return 0;
}