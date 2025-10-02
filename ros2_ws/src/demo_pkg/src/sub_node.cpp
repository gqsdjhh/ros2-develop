// rclcpp库
#include "rclcpp/rclcpp.hpp"
// 基本消息类型库
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

// 这样在下文可以使用1000ms这种表示方式
using namespace std::chrono_literals;

// 占位符,下面会详细说
using std::placeholders::_1;

// 创建一个类节点，起名叫做KFCNode,继承自Node,这样就能使用Node所有的功能了
class KFCNode : public rclcpp::Node
{
public:
    // 构造函数,第一个参数为节点名称, 并初始化count为1
    KFCNode(std::string name) : Node(name), count(1)
    {
        // 打印KFC的自我介绍
      	// c_str()函数是string类的一个函数，作用是把string类型转化为char类型(%s要求是一个字符串)
        RCLCPP_INFO(this->get_logger(), "大家好, 我是%s的服务员.",name.c_str());
        
        // 创建发布者, 发布hamburger, 发布的消息类型为<std_msgs::msg::String>
      	// 格式: 发布者名字 = this->create_publisher<要发布的话题类型>("要发布的话题名称", 通信Qos);
        pub_hamburger = this->create_publisher<std_msgs::msg::String>("hamburger", 10);
        
        // 创建发布者, 发布advertisement
        pub_advertisement = this->create_publisher<std_msgs::msg::String>("advertisement", 10);
        
        // 创建定时器,每5000ms发布一个广告
      	// 格式: 定时器名字 = his->create_wall_timer(1000ms, std::bind(&定时器回调函数, this));
        advertisement_timer = this->create_wall_timer(5000ms, std::bind(&KFCNode::advertisement_timer_callback, this));
        
        // 创建订阅者,订阅money
      	// 格式: 订阅者名字 = this->create_subscription<要订阅的话题类型>("要订阅的话题名称", 通信Qos, std::bind(&订阅者回调函数, this, _1));
      	// std::bind()是干啥的呢? 举个例子: 
      	// 		auto f = std::bind(fun, placeholders::_2, placeholders::_1, 80);
      	// 		f(60,70) 等效于 fun(70, 60, 80) 
      	// 还记得前文提到的占位符吗,placeholders::_1 就是f(60,70) 中的那个参数"1"
        sub_money = this->create_subscription<std_msgs::msg::UInt32>("money_of_hamburger", 10, std::bind(&KFCNode::money_callback, this, _1));
    }
private:
    // 定义一个汉堡售出计数器
  	// 在32位系统中size_t是4字节的，在64位系统中，size_t是8字节的，这样利用该类型可以增加程序移植性。
    size_t count;

    // 声明一个定时器
    rclcpp::TimerBase::SharedPtr advertisement_timer;

    // 声明一个发布者,用于发布汉堡
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_hamburger;
    
    // 声明一个订阅者,用于收钱
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_money;

    // 声明一个发布者,用于发布广告
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_advertisement;

    // 广告定时器回调函数(无参数)
    void advertisement_timer_callback()
    {
      	// 定义一个String类型的字符串, 其中字符串存在.data中, %s使用时别忘了使用.c_str()转换为char类型.
        auto str_advertisement = std_msgs::msg::String();
        str_advertisement.data = "大鸡腿降价啦";
        RCLCPP_INFO(this->get_logger(), "KFC发布了一个广告:%s", str_advertisement.data.c_str());
        pub_advertisement->publish(str_advertisement);
    }
    
    // 收钱订阅者回调函数(有参数, 参数类型跟上面订阅者订阅的参数类型相同, 注意要加上::SharedPtr, 因为传进来的是一个指针)
    void money_callback(const std_msgs::msg::UInt32::SharedPtr msg)
    {
        // 如果收到了十元钱,才发布汉堡. 订阅的信息在msg->data中
        if(msg->data == 10)
        {
            RCLCPP_INFO(this->get_logger(), "收款 %d 元", msg->data);

            // 字符串流
            auto str_hamburger_num = std_msgs::msg::String();
            str_hamburger_num.data = "第" + std::to_string(count++) + "个汉堡";
            RCLCPP_INFO(this->get_logger(), "这是我卖出的%s", str_hamburger_num.data.c_str());
            
            // 发布字符串流
          	// 发布就这么写 "发布器->publish(要发布的);", 简单吧
            pub_hamburger->publish(str_hamburger_num);
        }
        
    }
};

int main(int argc, char **argv)
{
    // 初始化rclcpp
    rclcpp::init(argc, argv);
    // 产生一个KFC的节点
    auto node = std::make_shared<KFCNode>("KFC");
  	// spin函数: 一旦进入spin函数，相当于它在自己的函数里面死循环了。只要回调函数队列里面有callback函数在，它就会马上去执行callback函数。如果没有的话，它就会阻塞，不会占用CPU。注意不要再spin后面放其他东西, 他们都不会执行的
    rclcpp::spin(node);
  	// 检测退出信号(ctrl+c)
    rclcpp::shutdown();
    return 0;
}