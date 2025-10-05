#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using namespace std::chrono_literals;

// 创建一个类节点，起名叫做KFCNode,继承自Node,这样就能使用Node所有的功能了
class KFCNode : public rclcpp::Node
{
public:
    // 构造函数,第一个参数为节点名称, 并初始化count为1
    KFCNode(std::string name) : Node(name), _count(1)
    {
        // 打印KFC的自我介绍
        RCLCPP_INFO(this->get_logger(), "大家好, 我是%s的服务员.",name.c_str());
        
        // 创建发布者, 发布hamburger
        _pub_hamburger = this->create_publisher<std_msgs::msg::String>("hamburger", 10);

        // 创建一个定时器
        _KFC_timer = this->create_wall_timer(1000ms, std::bind(&KFCNode::Timer_callback, this));
    }
private:
    // 用于记录汉堡的数量
    size_t _count;
    
    // 声明一个字符串类型的消息
    std_msgs::msg::String _str_hamburger_num;

    // 声明一个定时器
    rclcpp::TimerBase::SharedPtr _KFC_timer;

    // 声明一个发布者,用于发布汉堡
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub_hamburger;

    void Timer_callback(){
        _str_hamburger_num.data = "第" + std::to_string(_count++) + "个汉堡";

        RCLCPP_INFO(this->get_logger(), "卖了%s", _str_hamburger_num.data.c_str());

        _pub_hamburger->publish(_str_hamburger_num);
    }
};

int main(int argc, char **argv)
{
    // 初始化rclcpp
    rclcpp::init(argc, argv);
    // 产生一个KFC的节点
    auto node = std::make_shared<KFCNode>("KFC");

    rclcpp::spin(node);
  	// 检测退出信号
    rclcpp::shutdown();
    return 0;
}

