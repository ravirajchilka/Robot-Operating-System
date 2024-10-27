#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"

const auto radius = std::uint16_t(15); 

class RPMsubNode : public rclcpp::Node
{
    private:
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        std_msgs::msg::UInt16 speed_msg = std_msgs::msg::UInt16();

        void sub_callback(const std_msgs::msg::UInt16& rpm_msg) 
        {
            speed_msg.data = rpm_msg.data*radius;
            publisher_->publish(speed_msg);
        }


    public:
        RPMsubNode() : Node("rpm_sub_node")
        {
            subscription_ = this->create_subscription<std_msgs::msg::UInt16>
            (
                "rpm_value", 10, std::bind(&RPMsubNode::sub_callback, this, std::placeholders::_1)
            );

            publisher_ = this->create_publisher<std_msgs::msg::UInt16>("speed_value", 10);
        }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPMsubNode>());
    rclcpp::shutdown();

    return 0;
}

