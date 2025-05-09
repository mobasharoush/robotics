#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <limits>

using namespace std::chrono_literals;

enum class RobotState { FORWARD, TURN_LEFT, TURN_RIGHT, STOP };

class BumpGoNode : public rclcpp::Node
{
public:
    BumpGoNode() : Node("bump_go_node"), state_(RobotState::FORWARD)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&BumpGoNode::scanCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(100ms, std::bind(&BumpGoNode::controlLoop, this));
        last_scan_time_ = this->now();
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = *msg;
        last_scan_time_ = this->now();
    }

    void controlLoop()
    {
        auto now = this->now();

        // Stop state if scan data not received for over 0.5s
        if ((now - last_scan_time_).seconds() > 0.5) {
            state_ = RobotState::STOP;
        } else if (!latest_scan_.ranges.empty()) {
            const auto [left, right, front_obstacle] = analyzeScan(latest_scan_);
            if (front_obstacle) {
                state_ = (right < left) ? RobotState::TURN_LEFT : RobotState::TURN_RIGHT;
            } else {
                state_ = RobotState::FORWARD;
            }
        }

        publishCommand();
    }

    std::tuple<float, float, bool> analyzeScan(const sensor_msgs::msg::LaserScan &scan)
    {
        float left_min = std::numeric_limits<float>::max();
        float right_min = std::numeric_limits<float>::max();
        bool front_blocked = false;

        const int total_ranges = scan.ranges.size();
        const float angle_increment = scan.angle_increment;
        const float angle_min = scan.angle_min;
        const float angle_max = scan.angle_max;

        // Consider -30° to +30° in front
        for (int i = 0; i < total_ranges; ++i) {
            float angle_deg = (angle_min + i * angle_increment) * 180.0 / M_PI;
            float range = scan.ranges[i];

            if (range < scan.range_min || range > scan.range_max) continue;

            if (angle_deg > -30.0 && angle_deg < 30.0) {
                front_blocked |= (range < 0.5);
                if (angle_deg >= 0) left_min = std::min(left_min, range);
                else right_min = std::min(right_min, range);
            }
        }

        return {left_min, right_min, front_blocked};
    }

    void publishCommand()
    {
        geometry_msgs::msg::Twist cmd;

        switch (state_) {
            case RobotState::FORWARD:
                cmd.linear.x = 0.2;
                break;
            case RobotState::TURN_LEFT:
                cmd.angular.z = 0.5;
                break;
            case RobotState::TURN_RIGHT:
                cmd.angular.z = -0.5;
                break;
            case RobotState::STOP:
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                break;
        }

        publisher_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan latest_scan_;
    rclcpp::Time last_scan_time_;
    RobotState state_;
};
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BumpGoNode>());
    rclcpp::shutdown();
    return 0;
}
