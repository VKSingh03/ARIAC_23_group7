#pragma once

# include <memory>
# include "rclcpp/rclcpp.hpp"
# include <ariac_msgs/msg/competition_state.hpp>
# include <ariac_msgs/msg/order.hpp>
# include <ariac_msgs/srv/submit_order.hpp>
# include <ariac_msgs/msg/bin_parts.hpp>
# include <ariac_msgs/msg/conveyor_parts.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>

# include "order_class.hpp"

// Class to read competition state, receive and submit orders
class CompetitorControlSystem:public rclcpp::Node 
{

public:
    // Constructor for class CompetitiorControlSystem
    CompetitorControlSystem(std::string node_name):Node(node_name)
    {   
        // Subscriber to competition state for Launching Competition
        competition_state_subscriber = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&CompetitorControlSystem::competition_state_callback, this, std::placeholders::_1));
        // Subscriber to receive ariac orders
        subscriber_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
        std::bind(&CompetitorControlSystem::order_callback, this, std::placeholders::_1));
        // Subscriber to Competition state for Order Submission
        end_competition_subscriber = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&CompetitorControlSystem::ending_competition_callback, this, std::placeholders::_1));
        // Subscriber to bin_status
        bin_state_subscriber = this->create_subscription<ariac_msgs::msg::BinParts>("/ariac/bin_parts", 10, 
        std::bind(&CompetitorControlSystem::bin_status_callback, this, std::placeholders::_1));
        // Subscriber to conveyor_status
        conveyor_state_subscriber = this->create_subscription<ariac_msgs::msg::ConveyorParts>("/ariac/conveyor_parts", 10, 
        std::bind(&CompetitorControlSystem::conveyor_status_callback, this, std::placeholders::_1));
    }

    // Functions to complete specific tasks 
    // Function to get quantity of a particular part in the Bin
    int bin_get_quantity_for_this_part(uint8_t bin, uint8_t part, uint8_t color);
    // Function to update quantity for the given part on bin
    void bin_set_quantity_for_this_part(uint8_t bin, uint8_t part, uint8_t color, int value);
    // Function to update quantity for the given part on Conveyor
    void conveyor_set_quantity_for_this_part(uint8_t part, uint8_t color, int value);
    // Function for completing orders
    void CompleteOrders();
    // Function for completing Kitting task
    bool CompleteKittingTask(KittingInfo task);
    // Function for completing Assembly task
    bool CompleteAssemblyTask(AssemblyInfo task);
    // Function for completing Combined task
    bool CompleteCombinedTask(CombinedInfo task);
    // Function to complete Insufficient Parts Challange
    bool InsufficientPartsChallange(OrderData current_order_);
    // Function to Submit Orders
    bool SubmitOrder(std::string order_id);
    // Function to convert Part type to string
    std::string PartTypetoString(uint8_t part_type);
    // Function to convert Part colour to string
    std::string PartColortoString(uint8_t part_color);
    // Function to convert Destination to string
    std::string DestinationtoString(uint8_t dest);
    // Function to conver Station to string
    std::string StationtoString(uint8_t station);
    //Kitting Task Functions: 
    void FloorRobotPlacePartOnKitTray(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part,int tray_id, uint8_t agv_no );
    void MoveAGVkitting(uint8_t agv, uint8_t destination);
    void MoveAGVAsComb(uint8_t agv, uint8_t station);
    bool FloorRobotPickBinPart(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part);
    void FloorRobotChangeGripper(std::string station, std::string gripper_type);
    void FloorRobotSendHome();
    bool FloorRobotPickandPlaceTray(int tray_id, uint8_t agv_no);

    // Assembly Task Functions: 
    void CeilingRobotSendHome();
    void CeilingRobotPickTrayPart(AssemblyInfo task);
    void CeilingRobotPickTrayPart(CombinedInfo task);
    // void CeilingRobotPlacePartInInsert();

    //Combined Task Fumctioms: 
    void CombinedTaskAssemblyUpdate(CombinedInfo task);
    int AGVAvailable();

private:
    // Subscribers: 
    // Subscriber to read competition state READY
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_subscriber; 
    // Subscriber to receive orders. 
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr subscriber_; 
    // Subscriber to read Competition State & Order Submission and to implement EndCompetition Service client
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr end_competition_subscriber; 
    // Subscriber to bin status
    rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_state_subscriber; 
    // Subscriber to Converyor status
    rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_state_subscriber; 
    // Subscriber to Floor Robot Gripper type
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;

    // Variables: 
    // Variable to store competition state 
    unsigned int competition_state_;
    // Vector to store received orders based on priority
    std::vector<OrderData> orders_;
    // Variable to store current order
    // OrderData current_order_(nullptr);
    // Gripper State
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    // Variable to store position of first priority order in the vector
    int first_priority_order{0};
    // Variable to keep of total orders received and pending. 
    int total_orders{0};
    // To store bin read status
    int bin_read{0};
    // To store conveyor read status
    int conveyor_read{0};
    // Data Structure to store and update bin data
    std::map<uint8_t, std::map<uint8_t, std::map<uint8_t, uint8_t>>> bin_dictionary = {
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
    std::map<uint8_t, std::map<uint8_t, uint8_t>>conveyor_dictionary = {
    {10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}};

    // Data structure to store kitting task plan
    std::map<uint8_t, std::pair<std::pair<uint8_t, uint8_t>, uint8_t>> kitting_part_details = {
    {1 , std::make_pair(std::make_pair(0, 0), NULL)}, 
    {2 , std::make_pair(std::make_pair(0, 0), NULL)}, 
    {3 , std::make_pair(std::make_pair(0, 0), NULL)}, 
    {4 , std::make_pair(std::make_pair(0, 0), NULL)}};

    // Data Structure to store Assembly task
    std::map<uint8_t, std::map<uint8_t, std::pair<uint8_t, uint8_t>>> assembly_part_details = {
    {1, {{1, std::make_pair(0, 0)}, {2, std::make_pair(0, 0)}, {3, std::make_pair(0, 0)}, {4, std::make_pair(0, 0)}}},
    {2, {{1, std::make_pair(0, 0)}, {2, std::make_pair(0, 0)}, {3, std::make_pair(0, 0)}, {4, std::make_pair(0, 0)}}},
    {3, {{1, std::make_pair(0, 0)}, {2, std::make_pair(0, 0)}, {3, std::make_pair(0, 0)}, {4, std::make_pair(0, 0)}}},
    {4, {{1, std::make_pair(0, 0)}, {2, std::make_pair(0, 0)}, {3, std::make_pair(0, 0)}, {4, std::make_pair(0, 0)}}}};
    
    // Subscriber Callbacks.
    // Subscriber Callback to competition state
    void competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback to receive orders
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);
    // Subscriber Callback for Ending Competition
    void ending_competition_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    // Subscriber Callback for reading bin status
    void bin_status_callback(const ariac_msgs::msg::BinParts::ConstSharedPtr msg);
    // Subscriber Callback for reading conveyor status
    void conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg);
    // Floor robot Gripper State Callback
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    
}; 