#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "filter.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class NoiseSignalSubscriber : public rclcpp::Node
{
public:
    NoiseSignalSubscriber(std::string name)
     : Node(name)
    {
        // 创建订阅者，订阅正弦波
        _sin_sub = this->create_subscription<std_msgs::msg::Float32>("sin_noise_signal", 10, std::bind(&NoiseSignalSubscriber::SinCallback, this, std::placeholders::_1));
        
        // 创建发布者，发布处理后的信号
        _median_filter_pub = this->create_publisher<std_msgs::msg::Float32>("median_filter_pub_signal", 10);

        _low_pass_filter_pub = this->create_publisher<std_msgs::msg::Float32>("low_pass_filter_pub_signal", 10);
    }
private:
    void SinCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std_msgs::msg::Float32 _median_filtered_sin_value;
        std_msgs::msg::Float32 _low_pass_filtered_sin_value;
        _low_pass_filtered_sin_value.data = _low_pass_filter.process(msg->data);
        _median_filtered_sin_value.data = _median_filter.process(msg->data);

        _median_filter_pub->publish(_median_filtered_sin_value);
        _low_pass_filter_pub->publish(_low_pass_filtered_sin_value);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sin_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _median_filter_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _low_pass_filter_pub;

    MedianFilter _median_filter;
    LowPassFilter _low_pass_filter;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NoiseSignalSubscriber>("filtered_signal_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}