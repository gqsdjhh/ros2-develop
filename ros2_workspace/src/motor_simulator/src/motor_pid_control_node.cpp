#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "motor_simulator/filter.hpp"
#include "motor_simulator/pid.hpp"

using namespace std::chrono_literals;

class MotorControlNode : public rclcpp::Node {
public:
    explicit MotorControlNode(std::string name)
        : Node(name), pid_controller_(1000.0f, -1000.0f, 500.0f, -500.0f){

        control_publisher_ = this->create_publisher<std_msgs::msg::Float64>("motor_simulator_control_input", 10);
        
        angle_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("simulated_motor_angle", 10, std::bind(&MotorControlNode::angle_callback, this, std::placeholders::_1));
        
        velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("simulated_motor_velocity", 10, std::bind(&MotorControlNode::velocity_callback, this, std::placeholders::_1));
        
        torque_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("simulated_motor_torque", 10, std::bind(&MotorControlNode::torque_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(10ms, std::bind(&MotorControlNode::publish_control_command, this));

        pid_controller_.SetPid(0.04f, 0.0001f, 0.0f);
    }

private:
    void publish_control_command() {
        // target_velocity_ += 1;

        // if(target_velocity_ >= 500){
        //     target_velocity_ = 200;
        // }

        pid_controller_.SetTarget(target_velocity_);

        std_msgs::msg::Float64 msg;
        msg.data = pid_controller_.PidCal(median_filter.process(current_velocity_));
        control_publisher_->publish(msg);
    }
    
    void angle_callback(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        current_angle_ = msg->data;
    }
    
    void velocity_callback(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        current_velocity_ = msg->data;
    }

    void torque_callback(const std_msgs::msg::Float64::ConstSharedPtr& msg) {
        current_torque_ = msg->data;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_subscriber_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    double current_angle_ = 0.0;
    double current_velocity_ = 0.0;
    double current_torque_ = 0.0;

    double target_angle_ = 0.0;
    double target_velocity_ = 500.0;

    PositionPid pid_controller_;

    MedianFilter median_filter;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>("motor_controller"));
    rclcpp::shutdown();
    return 0;
}
