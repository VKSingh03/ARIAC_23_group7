#pragma once

# include <memory>
# include "rclcpp/rclcpp.hpp"
# include <ariac_msgs/msg/competition_state.hpp>
# include <ariac_msgs/msg/order.hpp>
# include <ariac_msgs/srv/submit_order.hpp>
#include <std_srvs/srv/trigger.hpp>

# include "order_class.hpp"

// Class to read competition state, receive and submit orders
class CompetitionStateSubscriber:public rclcpp::Node 
{

public:
    // Constructor for class CompetitionStateSubscriber
    CompetitionStateSubscriber(std::string node_name):Node(node_name)
    {   
        // Subscriber to competition state for Launching Competition
        competition_state_subscriber = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&CompetitionStateSubscriber::competition_state_callback, this, std::placeholders::_1));
        // Subscriber to receive ariac orders
        subscriber_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
        std::bind(&CompetitionStateSubscriber::order_callback, this, std::placeholders::_1));
        // Subscriber to Competition state for Order Submission
        order_submission_subscriber = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&CompetitionStateSubscriber::order_submission_callback, this, std::placeholders::_1));
        // Subscriber to Competition state for Order Submission
        end_competition_subscriber = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&CompetitionStateSubscriber::ending_competition_callback, this, std::placeholders::_1));
    }

private:
    // attributes
    // Subscriber to read competition state READY
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_subscriber; 
    // Vector to store received orders based on priority
    std::vector<OrderData> orders_;
    // Subscriber to receive orders. 
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr subscriber_; 
    // Subscriber to read competition state ORDER_ANNOUNCEMENT_DONE
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr order_submission_subscriber; 
    // Subscriber to read Competition State & Order Submission and to implement EndCompetition Service client
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr end_competition_subscriber; 
    // Variable to store first position of priority order in the vector
    int first_priority_order{0};
    // Variable to keep of total orders received and pending. 
    int total_orders{0};

    // methods
    // Subscriber Callback to competition state
    void competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback to receive orders
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);
    // Subscriber Callback for Order submission
    void order_submission_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback for Ending Competition
    void ending_competition_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
}; 