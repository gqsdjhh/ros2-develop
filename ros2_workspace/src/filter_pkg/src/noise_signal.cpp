#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <random> 

using namespace std::chrono_literals;

class NoiseSignalPublisher : public rclcpp::Node
{
public:
    NoiseSignalPublisher(std::string name, double noise_min, double noise_max) : Node(name)
    {
        // 创建发布者
        _sin_pub = this->create_publisher<std_msgs::msg::Float32>("sin_noise_signal", 10);
        
        // 设置定时器，发布频率约为500Hz（周期2ms）
        _timer = this->create_wall_timer(2ms, std::bind(&NoiseSignalPublisher::timer_callback, this));
        
        // 记录节点启动时间
        _start_time = this->now();

        noise = std::uniform_real_distribution<double>(noise_min, noise_max);

        rand_engine.seed(std::time(nullptr));
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

        auto noise_value = noise(rand_engine);
        sin.data += noise_value; // 添加噪声，范围[-0.2, 0.2]

        _sin_pub->publish(sin);
    }
    
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _sin_pub;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Time _start_time;
    std::default_random_engine rand_engine;  
    std::uniform_real_distribution<double> noise;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NoiseSignalPublisher>("noise_signal_publisher", -0.2, 0.2);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}