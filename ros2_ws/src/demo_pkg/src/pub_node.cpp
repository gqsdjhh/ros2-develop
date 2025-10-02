#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

// 这样就能使用1000ms这种表示方式
using namespace std::chrono_literals;

//占位符
using std::placeholders::_1;
using std::placeholders::_2;

// 创建一个类节点，名字叫做CustomerNode,继承自Node.
class CustomerNode : public rclcpp::Node
{
public:
    // 构造函数,第一个参数为节点名称
    CustomerNode(std::string name) : Node(name)
    {
        // 打印Customer的自我介绍
        RCLCPP_INFO(this->get_logger(), "大家好,我是一个%s.",name.c_str());

        // 创建订阅者,订阅hamburger
        // 占位符还记得吗? 复习一下, 此处的_1 表示const std_msgs::msg::String::SharedPtr msg
        sub_hamburger = this->create_subscription<std_msgs::msg::String>("hamburger", 10, std::bind(&CustomerNode::hamburger_callback, this, _1));
        
        // 创建订阅者,订阅advertisement
        sub_advertisement = this->create_subscription<std_msgs::msg::String>("advertisement", 10, std::bind(&CustomerNode::advertisement_callback, this, _1));
        
        // 创建定时器,每1000ms饿一次
        hungry_timer = this->create_wall_timer(1000ms, std::bind(&CustomerNode::hungry_timer_callback, this));

        // 创建发布者,发布money
        pub_money = this->create_publisher<std_msgs::msg::UInt32>("money_of_hamburger", 10);
        
        // 给money赋值
        money.data = 10;

        //第一次给钱
        pub_money->publish(money);
        RCLCPP_INFO(this->get_logger(), "我饿了, 我要吃汉堡! 付款 %d 元", money.data);

    }
private:
    // 新建一张钱
    std_msgs::msg::UInt32 money;

    // 声明一个定时器
    rclcpp::TimerBase::SharedPtr hungry_timer;

    // 声明一个订阅者,用于订阅发出的汉堡
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_hamburger;

    // 声明一个发布者,用于给KFC钱
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_money;

    // 声明一个订阅者,用于订阅广告
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_advertisement;
    
    // 汉堡订阅者回调函数
    void hamburger_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "这是我吃的 %s ", msg->data.c_str());
    }

    // 饥饿定时器回调函数
    void hungry_timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "我又饿了, 还想再吃一个! 付款 %d 元", money.data);
        pub_money->publish(money);
    }

    // 广告订阅者回调函数
    void advertisement_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "我收到了一条广告: %s ", msg->data.c_str());
    }
};

int main(int argc, char **argv)
{
    //初始化rclcpp
    rclcpp::init(argc, argv);
    //产生一个Customer的节点
    auto node = std::make_shared<CustomerNode>("Customer");
    //运行节点，并检测退出信号
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}