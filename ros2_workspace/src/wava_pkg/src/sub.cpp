#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class WaveSubscriber : public rclcpp::Node
{
public:
    WaveSubscriber(std::string name) : Node(name), _sin_value(0.0f),  _square_value(0.0f), _sin_received(false), _square_received(false) 
    {
        // 创建订阅者，分别订阅正弦波和方波信号
        _sin_sub = this->create_subscription<std_msgs::msg::Float32>("sin_signal", 10, std::bind(&WaveSubscriber::sin_callback, this, std::placeholders::_1));
        _square_sub = this->create_subscription<std_msgs::msg::Float32>("square_signal", 10, std::bind(&WaveSubscriber::square_callback, this, _1));
        
        // 创建发布者，发布处理后的信号
        _pub = this->create_publisher<std_msgs::msg::Float32>("pub_signal", 10);
    }
private:
    void sin_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        _sin_value = msg->data;
        process_signals();
    }

    void square_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        _square_value = msg->data;
        process_signals();
    }

    void process_signals()
    {
        // 确保两个信号都已接收到
        if (_sin_received && _square_received)
        {
            std_msgs::msg::Float32 sub_msg;
            // 判断正弦信号与方波信号是否同号
            if ((_sin_value >= 0 && _square_value >= 0) || (_sin_value < 0 && _square_value < 0))
            {
                sub_msg.data = _sin_value;
            }
            else
            {
                sub_msg.data = 0.0f;
            }
            _pub->publish(sub_msg);
            // 重置接收标记，准备下一次处理
            _sin_received = false;
            _square_received = false;
        }
        else if (!_sin_received)
        {
            _sin_received = true;
        }
        else if (!_square_received)
        {
            _square_received = true;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sin_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _square_sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub;

    float _sin_value;
    float _square_value;
    bool _sin_received;
    bool _square_received;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaveSubscriber>("wave_subscriber");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}