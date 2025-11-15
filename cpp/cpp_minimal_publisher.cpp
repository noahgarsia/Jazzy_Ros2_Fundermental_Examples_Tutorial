// important the necssary libraries
#include "rclcpp/rclcpp.hpp" // the libary to create a ros2 node
#include "std_msgs/msg/string.hpp" // the libary to tell what data type will be used

using namespace std::chrono_literals; // allows to use time literals like 500ms

class MinimalPublisher : public rclcpp::Node
// create a class that inherits from rclcpp::Node so it can act as a ROS2 node
{
public:
    MinimalPublisher() : Node("minimal_cpp_publisher"), counter_(0)
    // constructor, it builds a node called minimal_cpp_publisher and initializes counter_ to 0
    {
        publisher_ = create_publisher<std_msgs::msg::String>
        ("/project_1_example_topic", 10);
        // creates a publisher that will publish string messages on the topic ccp_example_topic
        // and if there is a subscriber and network issues occur, it will hold a backlog of 10
        // in its publishing queue/buffer

        timer_ = create_wall_timer(
            500ms,
            std::bind(&MinimalPublisher::timer_callback, this));
        // combines a timer with a callback function so every 500ms
        // the timer_callback function is called (publishing at 2Hz)

        RCLCPP_INFO(get_logger(), "Publishing at 2HZ");
        // update the log with the publishing rate
    }

    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        // Declares a message variable and sets its type to std_msgs::msg::String,
        // showing the type of data this publisher will send.

        message.data = "Hello World!" + std::to_string(counter_++);
        // Sets the content of the message. This adds the current loop iteration
        // so each message is unique.

        publisher_->publish(message);
        // Publishes the message to the topic that was created earlier.
    }

private:
    size_t counter_;
    // A counter that increases every time the timer_callback is called,
    // used to number each published message.

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // Shared pointer to the publisher object so the whole class can use it

    rclcpp::TimerBase::SharedPtr timer_;
    // Shared pointer to the timer so it keeps firing the callback
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto minimal_cpp_publisher_node = std::make_shared<MinimalPublisher>();
    rclcpp::spin(minimal_cpp_publisher_node);

    rclcpp::shutdown();

    return 0;
}
