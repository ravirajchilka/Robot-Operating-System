#include "rclcpp/rclcpp.hpp"
#include "udemy_ros2_pkg/srv/odd_even_check.hpp"

typedef udemy_ros2_pkg::srv::OddEvenCheck OddEvenCheckSrv;

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);

    auto service_client_node = rclcpp::Node::make_shared("odd_even_check_cient_node");
    auto client = service_client_node->create_client<OddEvenCheckSrv>("odd_even_check");
    auto request = std::make_shared<OddEvenCheckSrv::Request>();

    std::cout << "Please type a number to check if it is odd or even: " << std::endl;
    std::cin >> request->number;

    client->wait_for_service();

    auto result = client->async_send_request(request);

    if(rclcpp::spin_until_future_complete(service_client_node,result) == rclcpp::FutureReturnCode::SUCCESS) {
        std::cout << "The number is: " << result.get()->decision << std::endl;
    } else {
        std::cout << "There was an error processing the request..." << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}




