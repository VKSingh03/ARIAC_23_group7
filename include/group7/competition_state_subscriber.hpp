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
    // Subscribers: 
    // Subscriber to read competition state READY
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_subscriber; 
    // Subscriber to receive orders. 
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr subscriber_; 
    // Subscriber to read competition state ORDER_ANNOUNCEMENT_DONE
    // rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr order_submission_subscriber; 
    // Subscriber to read Competition State & Order Submission and to implement EndCompetition Service client
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr end_competition_subscriber; 
    // Subscriber to bin status
    rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_state_subscriber; 
    // Subscriber to Converyor status
    // rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_state_subscriber; 
    // Subscriber to Floor Robot Gripper type
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;

    // Variables: 
    // Variable to store competition state 
    unsigned int competition_state_;
    // Vector to store received orders based on priority
    std::vector<OrderData> orders_;
    // Variable to store current order
    OrderData current_order_;
    // Gripper State
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    // Variable to store position of first priority order in the vector
    int first_priority_order{0};
    // Variable to keep of total orders received and pending. 
    int total_orders{0};
    // To store bin read status
    int bin_read{0};
    // Data Structure to store and update bin data
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
    // Data Structure to store and update conveyor data
    std::map<int, std::map<int, int>>conveyor_dictionary = {
    {10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}};

    // Data structure to store kitting task plan
    std::map<int, std::pair<std::pair<int, int>, int>> kitting_part_details = {
    {1 , std::make_pair(std::make_pair(0, 0), None)}, 
    {2 , std::make_pair(std::make_pair(0, 0), None)}, 
    {3 , std::make_pair(std::make_pair(0, 0), None)}, 
    {4 , std::make_pair(std::make_pair(0, 0), None)}};

// }
    // Data Structure to store Kitting task
    // std::map<
    // Data Structure to store Assembly task

    // Data Structure to store Combined task

    // Subscriber Callbacks.
    // Subscriber Callback to competition state
    void competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback to receive orders
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);
    // // Subscriber Callback for Order submission
    // void order_submission_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback for Ending Competition
    void ending_competition_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback for reading bin status
    void bin_status_callback(const ariac_msgs::msg::BinParts::ConstSharedPtr msg);
    // Subscriber Callback for reading conveyor status
    void conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg);
    // Floor robot Gripper State Callback
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    
    // Functions to complete specific tasks 
    // Function to get quantity of a particular part in the Bin
    int get_quantity_for_this_part(int bin, int part, int color);
    // Function to update quantity for the given part on bin
    void set_quantity_for_this_part(int bin, int part, int color, int value);
    // Function for completing orders
    void CompleteOrders();
    // Function for completing Kitting task
    void CompleteKittingTask();
    // Function for completing Assembly task
    void CompleteAssemblyTask();
    // Function for completing Combined task
    void CompleteCombinedTask();

    //Kitting Task Functions: 
    void FloowRobtPlacePartOnKitTray();
    void MoveAGV();
    bool FloorRobotPickBinPart();
    void FloorRobotChangeGripper();
    bool FloorRobotSendHome();
    void InsufficientPartsChallange();



}; 