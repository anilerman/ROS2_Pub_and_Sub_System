#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <random>

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher()
        : Node("trajectory_publisher"), radius_(5.0), frequency_ms_(10)
    {
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("trajectory", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        // Timer period in ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(frequency_ms_),
            std::bind(&TrajectoryPublisher::publish_trajectory, this));

        path_.header.frame_id = "map";
    }

private:
    void publish_trajectory()
    {
        auto pose_message = geometry_msgs::msg::PoseStamped();

        // Calculate elapsed time
        double elapsed_time = this->now().seconds();

        // Generate base circular trajectory
        pose_message.pose.position.x = radius_ * cos(elapsed_time);
        pose_message.pose.position.y = radius_ * sin(elapsed_time);

        // Add random noise
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> noise_dist(0.0, 0.2);
        pose_message.pose.position.x += noise_dist(gen);
        pose_message.pose.position.y += noise_dist(gen);

        pose_message.header.frame_id = "map";
        pose_message.header.stamp = this->now();

        // Publish noisy Pose message
        pose_publisher_->publish(pose_message);

        // Add position to Path message
        path_.poses.push_back(pose_message);
        path_.header.stamp = this->now();

        // Publish Path message
        path_publisher_->publish(path_);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double radius_;
    int frequency_ms_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
