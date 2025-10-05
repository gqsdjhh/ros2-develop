#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class CustomerNode : public rclcpp::Node
{
public:
    // 构造函数
    CustomerNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好,我是一个%s.",name.c_str());

        // 创建订阅者,订阅hamburger
        _sub_hamburger = this->create_subscription<std_msgs::msg::String>("hamburger", 10, std::bind(&CustomerNode::hamburger_callback, this, _1));
    }
private:
    // 声明一个订阅者,用于订阅发出的汉堡
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub_hamburger;

    // 汉堡订阅者回调函数
    void hamburger_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "这是我吃的%s ", msg->data.c_str());
    }
}; 

int main(int argc, char **argv)
{
    //初始化rclcpp
    rclcpp::init(argc, argv);
    //产生一个Customer的节点
    auto node = std::make_shared<CustomerNode>("big_stomach");
    //运行节点，并检测退出信号
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

