#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "udemy_ros2_pkg/action/navigate.hpp"


typedef udemy_ros2_pkg::action::Navigate NavigateAction; // Navigate is the file name
typedef rclcpp_action::ClientGoalHandle<NavigateAction> GoalHandle;

class NavigateActionClientNode : public rclcpp::Node
{
    public:
        NavigateActionClientNode() : Node("navigate_action_client_node")
        {
            action_client_ = rclcpp_action::create_client<NavigateAction>(
                this, "navigate"
            ); 
            prompt_user_for_goal();
        }

    private:
        rclcpp_action::Client<NavigateAction>::SharedPtr action_client_;    
        
        void prompt_user_for_goal() 
        {
            using std::placeholders::_1;
            using std::placeholders::_2;

            auto goal_msg = NavigateAction::Goal();
            std::cout << "Enter a X coordinate to travel to: ";
            std::cin >> goal_msg.goal_point.x;

            std::cout << "Enter a y coordinate to travel to: ";
            std::cin >> goal_msg.goal_point.y;

            std::cout << "Enter a x coordinate to travel to: ";
            std::cin >> goal_msg.goal_point.z;

            this->action_client_->wait_for_action_server();
            std::cout << "sending goal" << std::endl;

            auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();    

            send_goal_options.goal_response_callback = std::bind(&NavigateActionClientNode::goal_response_callback,this,_1);
            send_goal_options.feedback_callback = std::bind(&NavigateActionClientNode::feedback_callback, this,_1,_2);
            send_goal_options.result_callback = std::bind(&NavigateActionClientNode::result_callback, this,_1);
        
            this->action_client_->async_send_goal(goal_msg, send_goal_options);
        }


        void goal_response_callback(GoalHandle::SharedPtr future)
        {
            auto goal_handle = future.get();
            if(!goal_handle) {
                std::cout <<  "Goal was rejected by the server" << std::endl;
            } else {
                std::cout <<  "Goal was accepted by the server, waiting for the results..." << std::endl;
            }
        }


        void feedback_callback(GoalHandle::SharedPtr future, const std::shared_ptr<const NavigateAction::Feedback> feedback)
        {
          (void) future;
          std::cout << "Feedback: " << feedback->distance_to_point << std::endl;
        }


        void result_callback(const GoalHandle::WrappedResult& result)
        {
            std::cout << "Time elapsed: " << result.result->elapsed_time << std::endl;
            switch(result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    std::cout << "Time_elapsed: " << result.result->elapsed_time << std::endl;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    std::cout << "Goald was aborted" << std::endl;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    std::cout << "Goald was canceled" << std::endl;
                    break;    
                default:
                    std::cout << "Unknown result code" << std::endl;
                    break;    
            }
            rclcpp::shutdown();
        }
};



int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateActionClientNode>());
    rclcpp::shutdown();

    return 0;
}

