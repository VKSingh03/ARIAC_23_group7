#pragma once

# include <memory>
# include "rclcpp/rclcpp.hpp"
# include <ariac_msgs/msg/competition_state.hpp>
# include <ariac_msgs/msg/order.hpp>
# include <ariac_msgs/srv/submit_order.hpp>
# include <ariac_msgs/msg/bin_parts.hpp>
# include <ariac_msgs/msg/conveyor_parts.hpp>
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
        // Subscriber to bin_status
        bin_state_subscriber = this->create_subscription<ariac_msgs::msg::BinParts>("/ariac/bin_parts", 10, 
        std::bind(&CompetitionStateSubscriber::bin_status_callback, this, std::placeholders::_1));
        // Subscriber to conveyor_status
        // conveyor_state_subscriber = this->create_subscription<ariac_msgs::msg::ConveyorParts>("/ariac/conveyor_parts", 10, 
        // std::bind(&CompetitionStateSubscriber::conveyor_status_callback, this, std::placeholders::_1));
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
    // Subscriber to bin status
    rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_state_subscriber; 
    // Subscriber to bin status
    // rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_state_subscriber; 
    // Variable to store first position of priority order in the vector
    int first_priority_order{0};
    // Variable to keep of total orders received and pending. 
    int total_orders{0};
    // To store bin read status
    int bin_read{0};
    // Data Structure to store bin data
    std::map<int, std::map<int, std::map<int, int>>> bin_dictionary = {
    {1, {{10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}}},
    {2, {{10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}}},
    {3, {{10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}}},
    {4, {{10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}}},
    {5, {{10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}}},
    {6, {{10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}}},
    {7, {{10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}}},
    {8, {{10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}}},
    };
    // Data Structure to store conveyor data
    std::map<int, std::map<int, int>>conveyor_dictionary = {
    {10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},{13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}};

    // methods
    // Subscriber Callback to competition state
    void competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback to receive orders
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);
    // Subscriber Callback for Order submission
    void order_submission_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback for Ending Competition
    void ending_competition_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback for reading bin status
    void bin_status_callback(const ariac_msgs::msg::BinParts::ConstSharedPtr msg);
    // Subscriber Callback for reading conveyor status
    // void conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg);
    // 
    int get_quantity_for_this_part(int bin, int part, int color);
    // Setter for 
    void set_quantity_for_this_part(int bin, int part, int color, int value);
}; 