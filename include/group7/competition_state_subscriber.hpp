#pragma once

# include <memory>
# include "rclcpp/rclcpp.hpp"
# include <ariac_msgs/msg/competition_state.hpp>
# include <ariac_msgs/msg/order.hpp>
# include <ariac_msgs/srv/submit_order.hpp>
# include <ariac_msgs/msg/bin_parts.hpp>
# include <ariac_msgs/msg/conveyor_parts.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <unistd.h>

#include <cmath>

#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/break_beam_status.hpp>
#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <chrono>

# include "order_class.hpp"

// Class to read competition state, receive and submit orders
class CompetitorControlSystem:public rclcpp::Node 
{

public:
    // Constructor for class CompetitiorControlSystem
    CompetitorControlSystem(std::string node_name):Node(node_name),
    floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
    ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),    
    planning_scene_()
    {
        // Use upper joint velocity and acceleration limits
        floor_robot_.setMaxAccelerationScalingFactor(1.0);
        floor_robot_.setMaxVelocityScalingFactor(1.0);

        ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
        ceiling_robot_.setMaxVelocityScalingFactor(1.0);

        rclcpp::SubscriptionOptions options;
        topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        options.callback_group = topic_cb_group_;

        // Subscriber to competition state for Launching Competition
        competition_state_subscriber = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 10, 
        std::bind(&CompetitorControlSystem::competition_state_callback, this, std::placeholders::_1));
        // Subscriber to receive ariac orders
        subscriber_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 10, 
        std::bind(&CompetitorControlSystem::order_callback, this, std::placeholders::_1));
        // Subscriber to bin_status
        bin_state_subscriber = this->create_subscription<ariac_msgs::msg::BinParts>("/ariac/bin_parts", 10, 
        std::bind(&CompetitorControlSystem::bin_status_callback, this, std::placeholders::_1));
        // Subscriber to conveyor_status
        conveyor_state_subscriber = this->create_subscription<ariac_msgs::msg::ConveyorParts>("/ariac/conveyor_parts", 10, 
        std::bind(&CompetitorControlSystem::conveyor_status_callback, this, std::placeholders::_1));
        // Subscriber to floor gripper state
        floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>("/ariac/floor_robot_gripper_state", 
        rclcpp::SensorDataQoS(), std::bind(&CompetitorControlSystem::floor_gripper_state_cb, this, std::placeholders::_1));
        // Subscriber to ceiling gripper state
        ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>("/ariac/ceiling_robot_gripper_state", 
        rclcpp::SensorDataQoS(), std::bind(&CompetitorControlSystem::ceiling_gripper_state_cb, this, std::placeholders::_1));        
        
        // Subscribers to sensors
        kts1_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/kts1_camera/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::kts1_camera_cb, this, std::placeholders::_1));

        kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::kts2_camera_cb, this, std::placeholders::_1));

        left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::left_bins_camera_cb, this, std::placeholders::_1));

        right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::right_bins_camera_cb, this, std::placeholders::_1));

        conveyor_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/conv_camera1/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::conveyor_camera_cb, this, std::placeholders::_1));

        conveyor_camera_counter_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/conv_camera1/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::conveyor_camera_counter_cb, this, std::placeholders::_1));

        breakbeam_start_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
            "/ariac/sensors/conv_beam1/status", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::breakbeam_start_cb, this, std::placeholders::_1));

        breakbeam_end_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
            "/ariac/sensors/conv_beam2/status", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::breakbeam_end_cb, this, std::placeholders::_1));

        breakbeam_start_counter_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
            "/ariac/sensors/conv_beam1/status", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::breakbeam_start_counter_cb, this, std::placeholders::_1));

        breakbeam_end_counter_sub_ = this->create_subscription<ariac_msgs::msg::BreakBeamStatus>(
            "/ariac/sensors/conv_beam2/status", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::breakbeam_end_counter_cb, this, std::placeholders::_1));

        agv1_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>("/ariac/agv1_status", 10, 
            std::bind(&CompetitorControlSystem::agv1_cb, this, std::placeholders::_1));

        agv2_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>("/ariac/agv2_status", 10, 
            std::bind(&CompetitorControlSystem::agv2_cb, this, std::placeholders::_1));

        agv3_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>("/ariac/agv3_status", 10, 
            std::bind(&CompetitorControlSystem::agv3_cb, this, std::placeholders::_1));

        agv4_sub_ = this->create_subscription<ariac_msgs::msg::AGVStatus>("/ariac/agv4_status", 10, 
            std::bind(&CompetitorControlSystem::agv4_cb, this, std::placeholders::_1));

        as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
            "/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::as1_state_cb, this, std::placeholders::_1));
        
        as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
            "/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::as2_state_cb, this, std::placeholders::_1));

        as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
            "/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::as3_state_cb, this, std::placeholders::_1));

        as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
            "/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::as4_state_cb, this, std::placeholders::_1));

        // Initialize service clients 
        // quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
        pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");
        floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
        floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
        ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");
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
    bool CompleteCombinedTask(OrderData current_order_);
    // Function to complete Insufficient Parts Challange
    bool InsufficientPartsChallange(OrderData current_order_);
    // Function to Submit Orders
    bool SubmitOrder(std::string order_id);
    // Function for Ending Competition
    void EndCompetition();
    // Function to convert Part type to string
    std::string PartTypetoString(uint8_t part_type);
    // Function to convert Part colour to string
    std::string PartColortoString(uint8_t part_color);
    // Function to convert Destination to string
    std::string DestinationtoString(uint8_t dest);
    // Function to conver Station to string
    std::string StationtoString(uint8_t station);
    
    //Kitting Task Functions: 
    bool FloorRobotPlacePartOnKitTray(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part,int tray_id, uint8_t agv_no );
    bool MoveAGVkitting(uint8_t agv, uint8_t destination);
    bool FloorRobotPickBinPart(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part);
    bool FloorRobotChangeGripper(std::string station, std::string gripper_type);
    void FloorRobotSendHome();
    bool FloorRobotPickandPlaceTray(int tray_id, int agv_no);
    bool FloorRobotConveyorPartspickup(int location);
    bool FloorRobotSetGripperState(bool enable);
    bool LockAGVTray(int agv_num);
    std::array<double,3> BinAvailableLocation(int location);
    
    // Assembly Task Functions: 
    void CeilingRobotSendHome();
    void CeilingRobotPickTrayPart(AssemblyInfo task);
    void CeilingRobotPickTrayPart(CombinedInfo task);
    bool MoveAGVAsComb(uint8_t agv, uint8_t station);
    // void CeilingRobotPlacePartInInsert();

    //Combined Task Functioms: 
    void CombinedTaskAssemblyUpdate(CombinedInfo task);
    int AGVAvailable(int station);
    int TrayAvailable(int station);
    bool CeilingRobotMoveToAssemblyStation(int station);
    bool CeilingRobotMovetoTarget();
    bool CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
    bool CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part);
    bool CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part);
    void CeilingRobotWaitForAttach(double timeout);
    bool CeilingRobotSetGripperState(bool enable);
    bool CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part);

private:

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

    // Robot Move Functions
    bool FloorRobotMovetoTarget();
    bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
    void FloorRobotWaitForAttach(double timeout);

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
    // Subscriber to Ceiling Robot Gripper type
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_; 
    //Sensors subscriber
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_camera_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_camera_counter_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_start_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_end_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_start_counter_sub_; 
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_end_counter_sub_; 
    //AGV status subscribers. Used for combined task
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv1_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv2_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv3_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv4_sub_;

    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;

    // Breakbeam part counter logic variables
    bool detected_first_breakbeam{false};
    bool detected_second_breakbeam{false};
    bool detected_conveyor_camera{false}; 

    // Sensor poses
    geometry_msgs::msg::Pose kts1_camera_pose_;
    geometry_msgs::msg::Pose kts2_camera_pose_;
    geometry_msgs::msg::Pose left_bins_camera_pose_;
    geometry_msgs::msg::Pose right_bins_camera_pose_;
    geometry_msgs::msg::Pose conveyor_camera_pose;
    geometry_msgs::msg::Pose conv_camera_pose_; 

    // Assembly States
    std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;

    // AGV Location
    int agv1_location; 
    int agv2_location; 
    int agv3_location; 
    int agv4_location; 

    // Sensor Callbacks
    bool kts1_camera_recieved_data = false; 
    bool kts2_camera_recieved_data = false; 
    bool left_bins_camera_recieved_data = false; 
    bool right_bins_camera_recieved_data = false; 
    bool conveyor_camera_received_data = false; 
    bool breakbeam_start_received_data = false; 
    bool breakbeam_end_received_data = false; 

    void kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void conveyor_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void conveyor_camera_counter_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    void breakbeam_start_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    void breakbeam_end_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    void breakbeam_start_counter_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    void breakbeam_end_counter_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);

    int breakbeam_one_counter{0};
    int breakbeam_two_counter{0};
    int ConveyorPartPickLocation();
    double conveyor_pickup_location_one = 0.65;
    double conveyor_pickup_location_two = -2.15;

    //AGV status callbacks
    void agv1_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    void agv2_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    void agv3_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    void agv4_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);

    // Helper Functions
    void LogPose(geometry_msgs::msg::Pose p);
    geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
    geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
    geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
    double GetYaw(geometry_msgs::msg::Pose pose);
    geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);

    void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
    void AddModelsToPlanningScene();

    geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Trays
    std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
    std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

    // Bins
    std::vector<ariac_msgs::msg::PartPose> left_bins_parts_;
    std::vector<ariac_msgs::msg::PartPose> right_bins_parts_;
    std::vector<ariac_msgs::msg::PartPose> conv_part_; 
    std::vector<ariac_msgs::msg::PartPose> conv_part_current;

    // MoveIt Interfaces
    moveit::planning_interface::MoveGroupInterface floor_robot_;
    moveit::planning_interface::MoveGroupInterface ceiling_robot_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

    // ARIAC Services
    // rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
    rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_;
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;

    // Variables: 
    // Variable to store competition state 
    unsigned int competition_state_;
    // Vector to store received orders based on priority
    std::vector<OrderData> orders_;
    // Variable to store current order
    // OrderData current_order_(nullptr);
    // Gripper State
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
    ariac_msgs::msg::Part floor_robot_attached_part_;
    ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
    ariac_msgs::msg::Part ceiling_robot_attached_part_;

    // Variable to store position of first priority order in the vector
    int first_priority_order{0};
    // Variable to keep of total orders received and pending. 
    int total_orders{0};
    // To store bin read status
    int bin_read{0};
    // To store conveyor read status
    int conveyor_read{0};
    int total_parts_for_conveyor{0};
    int conv_part_counter_breakbeam1{0};
    
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

    // Data structure to store kitting task plan // (type,color), bin 
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
    // Subscriber Callback for reading bin status
    void bin_status_callback(const ariac_msgs::msg::BinParts::ConstSharedPtr msg);
    // Subscriber Callback for reading conveyor status
    void conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg);
    // Floor robot Gripper State Callback
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    // Ceiling robot Gripper State Callback
    void ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

    void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

    std::map<std::string, double> rail_positions_ = {
        {"agv1", -4.5},
        {"agv2", -1.2},
        {"agv3", 1.2},
        {"agv4", 4.5},
        {"left_bins", 3}, 
        {"right_bins", -3}
    };
    // Joint value targets for kitting stations
    std::map<std::string, double> floor_kts1_js_ = {
        {"linear_actuator_joint", 4.0},
        {"floor_shoulder_pan_joint", 1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}
    };

    std::map<std::string, double> floor_kts2_js_ = {
        {"linear_actuator_joint", -4.0},
        {"floor_shoulder_pan_joint", -1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}
    };

    std::map<std::string, double> floor_conveyor_parts_pickup_1 = {
        {"linear_actuator_joint", -0.5},
        {"floor_shoulder_pan_joint", 3.14},
        {"floor_shoulder_lift_joint", -1.0},
        {"floor_elbow_joint", 2.13},
        {"floor_wrist_1_joint", -2.76},
        {"floor_wrist_2_joint", -1.51},
        {"floor_wrist_3_joint", 0.0}
    };

    std::map<std::string, double> floor_conveyor_parts_pickup_2 = {
        {"linear_actuator_joint", 2.3},
        {"floor_shoulder_pan_joint", 3.14},
        {"floor_shoulder_lift_joint", -1.0},
        {"floor_elbow_joint", 2.13},
        {"floor_wrist_1_joint", -2.76},
        {"floor_wrist_2_joint", -1.51},
        {"floor_wrist_3_joint", 0.0}
    };

    std::map<std::string, double> ceiling_as1_js_ = {
        {"gantry_x_axis_joint", 1},
        {"gantry_y_axis_joint", -3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as2_js_ = {
        {"gantry_x_axis_joint", -4},
        {"gantry_y_axis_joint", -3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as3_js_ = {
        {"gantry_x_axis_joint", 1},
        {"gantry_y_axis_joint", 3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    std::map<std::string, double> ceiling_as4_js_ = {
        {"gantry_x_axis_joint", -4},
        {"gantry_y_axis_joint", 3},
        {"gantry_rotation_joint", 1.571},
        {"ceiling_shoulder_pan_joint", 0},
        {"ceiling_shoulder_lift_joint", -2.37},
        {"ceiling_elbow_joint", 2.37},
        {"ceiling_wrist_1_joint", 3.14},
        {"ceiling_wrist_2_joint", -1.57},
        {"ceiling_wrist_3_joint", 0}
    };

    // Constants
    double kit_tray_thickness_ = 0.01;
    double drop_height_ = 0.002;
    double pick_offset_ = 0.003;
    double battery_grip_offset_ = -0.05;

    std::map<int, std::string> part_types_ = {
        {ariac_msgs::msg::Part::BATTERY, "battery"},
        {ariac_msgs::msg::Part::PUMP, "pump"},
        {ariac_msgs::msg::Part::REGULATOR, "regulator"},
        {ariac_msgs::msg::Part::SENSOR, "sensor"}
    };

    std::map<int, std::string> part_colors_ = {
        {ariac_msgs::msg::Part::RED, "red"},
        {ariac_msgs::msg::Part::BLUE, "blue"},
        {ariac_msgs::msg::Part::GREEN, "green"},
        {ariac_msgs::msg::Part::ORANGE, "orange"},
        {ariac_msgs::msg::Part::PURPLE, "purple"},
    };

    // Part heights
    std::map<int, double> part_heights_ = {
        {ariac_msgs::msg::Part::BATTERY, 0.04},
        {ariac_msgs::msg::Part::PUMP, 0.12},
        {ariac_msgs::msg::Part::REGULATOR, 0.07},
        {ariac_msgs::msg::Part::SENSOR, 0.07}
    };

    // Quadrant Offsets
    std::map<int, std::pair<double, double>> quad_offsets_ = {
        {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
    };

    std::array<std::array<double, 3>, 18> right_bin{{
        {1.07651, -0.235, -0.206007},
        {1.07651, -0.415, -0.206007},
        {1.07666, -0.595001, -0.206007},
        {1.07652, -0.235, -0.386007},
        {1.07675, -0.415, -0.386007},
        {1.07651, -0.595, -0.386007},
        {1.07675, -0.234999, -0.566007},
        {1.07651, -0.415, -0.566007},
        {1.07651, -0.595, -0.566007},
        {1.07651, 0.51502, -0.206},
        {1.07651, 0.33502, -0.206},
        {1.07652, 0.15502, -0.206},
        {1.07651, 0.51502, -0.386},
        {1.07651, 0.33502, -0.386},
        {1.07651, 0.15502, -0.386},
        {1.07651, 0.51502, -0.566},
        {1.07652, 0.33502, -0.566},
        {1.07651, 0.15502, -0.566}}} ;

    std::array<std::array<double, 3>, 18> left_bin{{
        {1.07651, 0.595, -0.206007},
        {1.07651, 0.415, -0.206007},
        {1.07678, 0.235, -0.206006},
        {1.07651, 0.595, -0.386007},
        {1.07691, 0.415, -0.386007},
        {1.07652, 0.235, -0.386007},
        {1.07682, 0.595001, -0.566007},
        {1.07651, 0.415, -0.566007},
        {1.07651, 0.235, -0.566007},
        {1.07652, -0.15498, -0.206},
        {1.07651, -0.33498, -0.206},
        {1.07652, -0.51498, -0.206},
        {1.07651, -0.15498, -0.386},
        {1.07651, -0.33498, -0.386},
        {1.07651, -0.51498, -0.386},
        {1.07651, -0.15498, -0.566},
        {1.07651, -0.33498, -0.566},
        {1.07651, -0.51498, -0.566}}} ;

}; 