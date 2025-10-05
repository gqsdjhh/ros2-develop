#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

using namespace std::chrono_literals;

class WavePublisher : public rclcpp::Node
{
public:
    WavePublisher(std::string name) : Node(name)
    {
        // 创建发布者，分别发布正弦波和方波
        _sin_pub = this->create_publisher<std_msgs::msg::Float32>("sin_signal", 10);
        _square_pub = this->create_publisher<std_msgs::msg::Float32>("square_signal", 10);
        
        // 设置定时器，发布频率约为500Hz（周期2ms）
        _timer = this->create_wall_timer(2ms, std::bind(&WavePublisher::timer_callback, this));
        
        // 记录节点启动时间
        _start_time = this->now();
    }

private:
    void timer_callback()
    {
        // 计算自启动以来的时间（秒）
        auto current_time = this->now() - _start_time;
        double t = current_time.seconds();
        
        // 生成10Hz正弦波
        std_msgs::msg::Float32 sin;
        sin.data = std::sin(2 * M_PI * 10 * t);
        _sin_pub->publish(sin);
        
        // 生成1Hz方波
        std_msgs::msg::Float32 square;
        square.data = (std::sin(2 * M_PI * 1 * t) >= 0) ? 1.0f : -1.0f;
        _square_pub->publish(square);
    }
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _sin_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _square_pub;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _start_time;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WavePublisher>("wave_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
