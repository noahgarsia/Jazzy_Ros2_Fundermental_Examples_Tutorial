//
// @__FILE__: src/cpp_minimal_subscriber.cpp
// @ Author: Noah Garsia
// @ Brief: A minimal C++ subscriber node example for ROS2 (GCC-13 compatible)
// @__DATE__ 2025-11-13
//

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node(
        "minimal_cpp_subscriber",
        rclcpp::NodeOptions()
            .use_intra_process_comms(false)   // 🔥 FIX #1 — disable intra-process comms
    )
    {
        // 🔥 FIX #2 — completely disable topic statistics
        rclcpp::SubscriptionOptions options;
        options.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/project_1_example_topic",   // <-- ONLY CHANGE
            rclcpp::QoS(10),
            std::bind(&MinimalSubscriber::topic_callback, this, _1),
            options
        );
    }

private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
        RCLCPP_INFO(
            this->get_logger(),
            "I heard: '%s'",
            msg.data.c_str()
        );
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MinimalSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
