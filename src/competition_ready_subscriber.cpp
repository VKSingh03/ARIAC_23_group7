# include "competition_ready_subscriber.hpp"
// # include "rclcpp/rclcpp.hpp"
// # include <ariac_msgs/msg/competition_state.hpp>
// # include <ariac_msgs/msg/order.hpp>
// # include <ariac_msgs/srv/submit_order.hpp>
// #include <std_srvs/srv/trigger.hpp>
// # include <memory>


void competition_ready_subscriber::competition_state_callback(const ariac_msgs::msg::CompetitionState::SharedPtr msg){
        if(msg->competition_state == ariac_msgs::msg::CompetitionState::READY){
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
            client = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result =client->async_send_request(request);
            result.wait();
            result.get()->success;
        }
    }

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto comp_state_subscriber = std::make_shared<competition_ready_subscriber>("CompetitionStateSubscriber");
    rclcpp::spin(comp_state_subscriber);
    rclcpp::shutdown();
}

