# include <memory>
# include "rclcpp/rclcpp.hpp"
# include <ariac_msgs/msg/competition_state.hpp>
# include <ariac_msgs/msg/order.hpp>
# include <ariac_msgs/srv/submit_order.hpp>
#include <std_srvs/srv/trigger.hpp>

class competition_ready_subscriber:public rclcpp::Node
{

public:
    competition_ready_subscriber(std::string node_name):Node(node_name)
    {
        competition_state_subscriber = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, std::bind(&competition_ready_subscriber::competition_state_callback, this, std::placeholders::_1));
    }

private:
    // attributes
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_subscriber;
    ariac_msgs::msg::CompetitionState msg;
    // rclcpp::Client<std_srvs::srv::Trigger>::ConstSharedPtr client;

    // methods
    void competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
};