#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include <iostream>
#include <memory>

class RPMpubNode : public rclcpp::Node 
{   
    private:
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::uint16_t rpmValue = std::uint16_t {12};
        rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_;

        void sub_callback(const std_msgs::msg::UInt16& msg)  
        {
            std::cout << "speed value received: "<< msg.data << std::endl;
        }

        void publish_rpm_value()
        {
            auto rpm = std_msgs::msg::UInt16();
            rpm.data = rpmValue++;
            std::cout << "rpm value sent: " << rpm.data << std::endl;
            publisher_->publish(rpm);
        }

     
    public:
        RPMpubNode() : Node("rpm_pub_node") 
        {
            publisher_ = this->create_publisher<std_msgs::msg::UInt16>("rpm_value",10);       
            timer_ = this->create_wall_timer
            (
                std::chrono::seconds(1),
                std::bind(&RPMpubNode::publish_rpm_value,this)
            );

             subscription_ = this->create_subscription<std_msgs::msg::UInt16>
            (
                "speed_value", 10, std::bind(&RPMpubNode::sub_callback, this, std::placeholders::_1)
            );
        }
};


int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPMpubNode>());
    rclcpp::shutdown();

    return 0;

}



