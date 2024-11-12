#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class TrajectorySubscriber : public rclcpp::Node {
public:
    TrajectorySubscriber()
        : Node("trajectory_subscriber"), filter_coefficient_(0.1) {
        
        // Subscribe to noisy pose topic
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "trajectory", 10, std::bind(&TrajectorySubscriber::filter_noisy_pose, this, std::placeholders::_1)
        );

        // Publisher for filtered pose
        filtered_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("filtered_trajectory", 10);
    }

private:
    void filter_noisy_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto filtered_pose_message = geometry_msgs::msg::PoseStamped();
        
        // Apply a simple low-pass filter
        filtered_pose_message.pose.position.x = previous_pose_.pose.position.x + filter_coefficient_ * (msg->pose.position.x - previous_pose_.pose.position.x);
        filtered_pose_message.pose.position.y = previous_pose_.pose.position.y + filter_coefficient_ * (msg->pose.position.y - previous_pose_.pose.position.y);

        // Update the previous pose for the next filtering step
        previous_pose_ = filtered_pose_message;

        filtered_pose_message.header.frame_id = "map";
        filtered_pose_message.header.stamp = this->now();

        // Publish the filtered pose
        filtered_pose_publisher_->publish(filtered_pose_message);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_publisher_;
    geometry_msgs::msg::PoseStamped previous_pose_;
    double filter_coefficient_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectorySubscriber>());
    rclcpp::shutdown();
    return 0;
}
