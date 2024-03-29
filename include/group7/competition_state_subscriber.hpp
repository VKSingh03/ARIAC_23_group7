/**
 *  @brief     This file contains the class definition of CompetitorControlSystem class which reads 
 * competition state, receives and submits orders, and performs required tasks based on the orders.
 *  @author    Vineet Singh
 *  @author    Krishna Hundekari
 *  @author    Ishan Tamrakar
 *  @author    Pranav Shinde
 *  @version   4.0
 *  @date      30-04-2023
 */

#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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

/**
 *
 * @class CompetitorControlSystem
 * @brief Manages the Complete Control of the robots for the Competition, 
 * including the Order completion and agility challenges.
 *
 */
class CompetitorControlSystem:public rclcpp::Node 
{

public:
    /**
     * @brief Constructor for a new Competitor Control System object
     * @param node_name ROS2 node name
     *
     */
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
            std::bind(&CompetitorControlSystem::kts1_camera_cb, this, std::placeholders::_1),options);

        kts2_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/kts2_camera/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::kts2_camera_cb, this, std::placeholders::_1),options);

        left_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/left_bins_camera/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::left_bins_camera_cb, this, std::placeholders::_1),options);

        right_bins_camera_sub_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
            "/ariac/sensors/right_bins_camera/image", rclcpp::SensorDataQoS(), 
            std::bind(&CompetitorControlSystem::right_bins_camera_cb, this, std::placeholders::_1),options);

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
        quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check");
        pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses");
        floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper");
        floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper");
        ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper");
    }

    /**
     * @brief method to get quantity of a particular part in the Bin
     *
     * @param bin the bin number key
     * @param part part the part type key
     * @param color color the part color key
     * @return returns an integer with the corresponding bin, parts, color
     */
    int bin_get_quantity_for_this_part(uint8_t bin, uint8_t part, uint8_t color);
    /**
     * @brief method to update quantity for the given part on bin
     *
     * @param bin the bin number key
     * @param part the part type key
     * @param color the part color key
     * @param value Value that is updated in the corresponding bin, part, color
     */
    void bin_set_quantity_for_this_part(uint8_t bin, uint8_t part, uint8_t color, int value);
    /**
     * @brief method to update quantity for the given part on Conveyor
     *
     * @param part the part type key
     * @param color the part color key
     * @param value the new quantity value
     */
    void conveyor_set_quantity_for_this_part(uint8_t part, uint8_t color, int value);
    /**
     * @brief method for completing orders
     *
     */
    bool CompleteOrders();
    /**
     * @brief method for completing Kitting task
     *
     * @param task object that contains all the information for the Kitting Task
     * @return returns true if the Kitting task was successfully completed
     */
    bool CompleteKittingTask(OrderData current_order_);
    /**
     * @brief method for completing Assembly task
     *
     * @param task object that contains all the information for the Assembly Task
     * @return returns true if the Assembly task was successfully completed
     */
    bool CompleteAssemblyTask(OrderData current_order_);
    /**
     * @brief method for completing Assembly task
     *
     * @param task object that contains all the information for the Assembly Task
     * @return returns true if the Assembly task was successfully completed
     */
    bool CompleteCombinedTask(OrderData current_order_);
    /**
     * @brief method to complete Insufficient Parts Challange
     *
     * @param current_order_ object that contains details of the current order
     * @return returns true if there were insufficient parts
     */
    bool InsufficientPartsChallange(OrderData current_order_);
    /***
     * @brief Method to start the Competition
     *
    */
    bool StartCompetition();
    /**
     * @brief method to Submit Orders
     *
     * @param order_id the ID of the order to submit
     * @return true if the service call to the client was successful
     * @return false if the service call to the client was not succesful
     */
    bool SubmitOrder(std::string order_id);
    /**
     * @brief method for Ending Competition and display on the console that the competition has ended
     *
     */
    void EndCompetition();
    /**
     * @brief method to convert Part type to corresponding string
     *
     * @param part_type the part type as uint8 value as stored in the order data object
     * @return std::string output of the corresponding part type as encoded in the ARIAC Order message
     */
    std::string PartTypetoString(uint8_t part_type);
    /**
     * @brief method to convert Part colour to corresponding string
     *
     * @param part_color the part color as uint8 value as stored in the order data object
     * @return std::string output of the corresponding part color as encoded in the ARIAC Order message
     */
    std::string PartColortoString(uint8_t part_color);
    /**
     * @brief method to convert Destination to corresponding string
     *
     * @param dest the destination as uint8 value as stored in the order data object
     * @return std::string output of the corresponding destination as encoded in the ARIAC Order message
     */
    std::string DestinationtoString(uint8_t dest);
    /**
     * @brief method to conver Station to corresponding string
     *
     * @param station the station as uint8 value as stored in the order data object
     * @return std::string output of the corresponding station as encoded in the ARIAC Order message
     */
    std::string StationtoString(uint8_t station);
    
    /**
     * @brief This method moves floor robot to AGV and places the part on kitting tray
     * @details This has been left blank intentionally
     * @param quadrant the quadrant on the kitting tray where the part is going to be placed
     * @param part the part that is going to be placed on the kitting tray
     * @param tray_id the kitting tray id that will be used
     * @param agv_no the avg number where the floor robot will place the part
     * @return true, if the placing of the part was succesful
     * @return false, if the gripper is not attached
     */ 
    bool FloorRobotPlacePartOnKitTray(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part,int tray_id, uint8_t agv_no, std::string order_id );
    /**
     * @brief This method will move the AGV to the final destination
     *
     * @param agv the AGV number that is supposed to be moved
     * @param destination the final destination where the AGV is supposed to go to
     * @return true, if the service call to the client to MoveAGV was succesful
     * @return false, uf the service call to the client to MoveAGV was unsuccesful
     */
    bool MoveAGV(uint8_t agv, uint8_t destination);
    /**
     * @brief method to check the bins for required part and picks up the required part
     * @details This part has been left blank intentionally
     * @param part the part that is supposed to be picked up 
     * @return true, if the required part was picked up
     * @return false, if the part was not found in the bins
     */
    bool FloorRobotPickBinPart(std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part);
    /**
     * @brief method to change the gripper type using a service call 
     *
     * @param station the kitting station where the gripper change is to be performed
     * @param gripper_type the name of the desired gripper type 
     * @return true, if the gripper was changed succesfully 
     * @return false, if the robot moves to an undesired location or if the service call to the client was unsuccesful
     */
    bool FloorRobotChangeGripper(std::string station, std::string gripper_type);
    /**
     * @brief method to move floor robot to home position
     *
     */
    void FloorRobotSendHome();
    /**
     * @brief method to make the floor robot find the required tray, pick and place it on the required AGV
     * @details Add details here
     * @param tray_id the tray that is required for the current order
     * @param agv_no the AGV number that is required to be used 
     * @return true, if the required tray was succesfully placed on the AGV
     * @return false, if the required tray was not found 
     */
    bool FloorRobotPickandPlaceTray(int tray_id, int agv_no);
    /**
     * @brief method to move floor robot to pick parts from conveyor 
     * @details diy later
     * @return true, if the part was succesfully picked up 
     * @return false, if unsuccesful
     */
    bool FloorRobotConveyorPartspickup(int location);
    /**
     * @brief method to activate the gripper
     * @details Check later for return values
     * @param enable the boolean that stores whether a gripper is on or off
     * @return true, if the gripper is activated
     * @return false, if the gripper was already enabled 
     */
    bool FloorRobotSetGripperState(bool enable);
    /**
     * @brief method to lock the tray onto the AGV
     *
     * @param agv_num the AGV number that is supposed to be locked
     * @return true, if the service call to the client to lock agv was succesful
     * @return false, if the service call to the client to lock agv was unsuccesful 
     */
    bool LockAGVTray(int agv_num);
    /**
     * @brief method to check available location for bins to place conveyor pickedup parts
     * @param location conveyor pickup location of part. (1 or 2)
     * @return x, y, z coordinate of available location
     */
    std::array<double,3> BinAvailableLocation(int location);
    
    /**
     * @brief method to move ceiling robot to home position
     * 
     */
    void CeilingRobotSendHome(); 

    /**
     * @brief Method to find Available agv for combined task
     * @param station on which combined will happen
     * @return agv number
     */
    int AGVAvailable(int station);
    // int TrayAvailable(int station);
    /**
     * @brief Takes station information and accordingly assigns joint value target to implement 
     * function to move ceiling robot to desired target.
     * 
     * @param station 
     * @return true if move action successful
     * @return false if invalid station
     */
    bool CeilingRobotMoveToAssemblyStation(int station);
    /**
     * @brief Function to move ceiling robot to desired location by accessing the Joint value target.
     */
    bool CeilingRobotMovetoTarget();
    /**
     * @brief Move floor robot in cartesian path
     * 
     * @param waypoints 
     * @param vsf 
     * @param asf 
     * @param avoid_collisions 
     * @return true 
     * @return false 
     */
    bool CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
    /**
     * @brief method to move a given part using ceiling robot
     * 
     * @param part the part that needs to be moved by the ceiling robot
     * @return true and carries out further steps
     * @return false 
     */
    bool CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part);
    /**
     * @brief verifies whether the ceiling robot has a part attached to it and the part is of correct description as per 
     * the order and then inserts it into the desired location at kitting station
     * 
     * @param station provides info about required assembly station
     * @param part details of the part to be assembled
     * @return true 
     * @return false 
     */
    bool CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part);
    /**
     * @brief method to provide a time delay for the part to be attached
     * 
     * @param timeout time delay to be provided
     */
    void CeilingRobotWaitForAttach(double timeout);
    /**
     * @brief method to activate the vacuum gripper of the robot.
     * 
     * @param enable 
     */
    bool CeilingRobotSetGripperState(bool enable);
    /**
     * @brief Move the part around to ensure sufficent collision to attach the part
     * 
     * @param station station no. at which action is carried out
     * @param part details of the part to be assembled
     * @return true 
     * @return false 
     */
    bool CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part);
    /**
     * @brief Move the sensor around at assembly insert to ensure sufficent collision to attach the part
     * 
     * @param station station no. at which action is carried out
     * @param part details of the part to be assembled
     * @return true 
     * @return false 
     */
    bool CeilingRobotWaitForAssembleSensor(int station, ariac_msgs::msg::AssemblyPart part);

private:

    //! Callback group
    /*!
      Callback group
    */
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    //! Callback group
    /*!
      Callback group
    */
    rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

    /**
       * @brief Move floor robot to a given target
       * @see FloorRobotMovetoTarget()
       * @return bool value - success of failure
       */
    bool FloorRobotMovetoTarget();
    /**
       * @brief Move floor robot in cartesian path
       * @see FloorRobotMoveCartesian()
       * @param waypoints set of waypoints that the robot should follow 
       * @param vsf the second argument.
       * @param asf the second argument.
       * @return bool value - success of failure
       */
    bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
    /**
       * @brief Move floor robot to attach part to its gripper
       * @see FloorRobotWaitForAttach()
       * @param timeout to stop the function if part not attached in this time. 
       */
    void FloorRobotWaitForAttach(double timeout);
    /**
       * @brief floor robot to attach pump to its gripper as pump behaves differently as compared to other parts
       * @see FloorRobotWaitForAttachPump()
       */
    void FloorRobotWaitForAttachPump(double timeout);

    /**
       * @brief Calls service calls to check faulty part. 
       * @see CheckFaultyPart()
       * @param order_id set of waypoints that the robot should follow 
       * @param quadrant the second argument
       * @return bool value - success of failure
       */
    bool CheckFaultyPart(std::string order_id, int quadrant);
    /**
       * @brief Move the faulty part and drop it in the bin
       * @see ThrowFaultyPartInBin()
       * @return bool value - success of failure
       */
    bool ThrowFaultyPartInBin();
    /**
       * @brief Returns location where the robot should pickup next part from conveyor
       * @see ConveyorPartPickLocation()
       * @return 1 or 2 for first or second location
       */
    int ConveyorPartPickLocation();
    /**
       * @brief Add Models to the MoveIt Planning Scene
       * @see AddModelToPlanningScene()
       * @param name name of the model of type std::string
       * @param mesh_file name of mesh file of type std::string
       * @param model_pose pose of the model of type geometry_msgs::msg::Pose
       */
    void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
    // void AddModelsToPlanningScene();

    //! A private variable
    /*!
      Variable to store agv assembly for part poses if priority order comes in between a assembly/combined order
    */
    std::vector<std::vector<ariac_msgs::msg::PartPose>> assembly_agv_part_poses; 

    // Subscribers: 

    //! Subscriber to competition state
    /*!
      Subscriber to competition state
    */
    rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_subscriber; 
    //! Subscriber to ARIAC Orders
    /*!
      Subscriber to ARIAC Orders
    */
    rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr subscriber_; 
    //! Subscriber to Bins status
    /*!
      Subscriber to bins parts status
    */
    rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_state_subscriber; 
    //! Subscriber to Converyor status
    /*!
      Subscriber to Converyor status
    */
    rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_state_subscriber; 
    //! Subscriber to Floor robot gripper state
    /*!
      Subscriber to Floor robot gripper state
    */
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
    //! Subscriber to Ceiling robot gripper state
    /*!
      Subscriber to Ceiling robot gripper state
    */
    rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_; 
    
    //Sensors subscriber
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts1_camera_sub_; /*!< Kit tray 1 camera subscriber */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kts2_camera_sub_; /*!< Kit tray 2 camera subscriber */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bins_camera_sub_; /*!< Left bins camera subscriber */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bins_camera_sub_; /*!< Right bins camera subscribe */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_camera_sub_; /*!< Conveyor camera sbscriber */
    rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr conveyor_camera_counter_sub_; /*!< Conveyor camera subscriber to keep count of parts */
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_start_sub_; /*!< First breakbeam subscriber */
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_end_sub_; /*!< Second breakbeam subscriber */
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_start_counter_sub_; /*!< First breakbeam subscriber to keep count of parts */
    rclcpp::Subscription<ariac_msgs::msg::BreakBeamStatus>::SharedPtr breakbeam_end_counter_sub_; /*!< Second breakbeam subscriber to keep count of parts */
    //AGV status subscribers. Used for combined task
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv1_sub_; /*!< Subscriber to AGV1 */
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv2_sub_; /*!< Subscriber to AGV2 */
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv3_sub_; /*!< Subscriber to AGV3 */
    rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv4_sub_; /*!< Subscriber to AGV4 */

    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_; /*!< Subscriber to AS1 station assembly poses */
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_; /*!< Subscriber to AS2 station assembly poses */
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_; /*!< Subscriber to AS3 station assembly poses */
    rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_; /*!< Subscriber to AS4 station assembly poses */

    // Breakbeam part counter logic variables
    bool detected_first_breakbeam{false}; /*!< Counter variable for first breakbeam */
    bool detected_second_breakbeam{false}; /*!< Counter variable for second breakbeam */
    bool detected_conveyor_camera{false}; /*!< Counter variable for conveyor camera */

    // Sensor poses
    geometry_msgs::msg::Pose kts1_camera_pose_; /*!< Stores Kit tray station 1 camera pose in world frame */
    geometry_msgs::msg::Pose kts2_camera_pose_; /*!< Stores Kit tray station 2 camera pose in world frame */
    geometry_msgs::msg::Pose left_bins_camera_pose_; /*!< Stores Left Bins camera pose in world frame */
    geometry_msgs::msg::Pose right_bins_camera_pose_; /*!< Stores Right Bins camera pose in world frame */
    // geometry_msgs::msg::Pose conveyor_camera_pose; /*!< Stores Conveyor camera pose in world frame */
    geometry_msgs::msg::Pose conv_camera_pose_; /*!< Stores Conveyor camera pose in world frame */

    // Assembly States
    std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;/*!< Map to Store Assembly station states */

    // AGV Location
    int agv1_location; /*!< Stores AGV1 current location */
    int agv2_location; /*!< Stores AGV2 current location */
    int agv3_location; /*!< Stores AGV3 current location */
    int agv4_location; /*!< Stores AGV4 current location */

    // Sensor Callbacks
    bool kts1_camera_recieved_data = false; /*!< Bool to store Kit tray station 1 camera data received or not */
    bool kts2_camera_recieved_data = false; /*!< Bool to store Kit tray station 2 camera data received or not */
    bool left_bins_camera_recieved_data = false; /*!< Bool to store left bins camera data received or not */
    bool right_bins_camera_recieved_data = false; /*!< Bool to store right bins camera data received or not */
    bool conveyor_camera_received_data = false; /*!< Bool to store canveyor camera data received or not */
    bool breakbeam_start_received_data = false; /*!< Bool to store breakbeam 1 data received or not */
    bool breakbeam_end_received_data = false; /*!< Bool to store breakbeam 2 data received or not */

    int breakbeam_one_counter{0}; /*!< Counter for breakbeam 1 parts detected */
    int breakbeam_two_counter{0}; /*!< Counter for breakbeam 2 parts detected */
    double conveyor_pickup_location_one = 0.65; /*!< Location for breakbeam1 location pickup from conveyor */
    double conveyor_pickup_location_two = -2.15; /*!< Location for breakbeam2 location pickup from conveyor */

    /**
       * @brief Subscriber Callback to ARIAC competition state
       * @see competition_state_callback()
       * @param msg a constant pointer of type ariac_msgs::msg::CompetitionState::ConstSharedPtr
       */
    void competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to ARIAC orders
       * @see order_callback()
       * @param msg a constant pointer of type ariac_msgs::msg::Order::SharedPtr
       */
    void order_callback(const ariac_msgs::msg::Order::SharedPtr msg);
    /**
       * @brief Subscriber Callback to ARIAC bins status
       * @see bin_status_callback()
       * @param msg a constant pointer of type ariac_msgs::msg::BinParts::ConstSharedPtr
       */
    void bin_status_callback(const ariac_msgs::msg::BinParts::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to ARIAC conveyor status
       * @see conveyor_status_callback()
       * @param msg a constant pointer of type ariac_msgs::msg::ConveyorParts::ConstSharedPtr
       */
    void conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Floor robot gripper state
       * @see floor_gripper_state_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::VacuumGripperState::ConstSharedPtr
       */
    void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Ceiling robot gripper state
       * @see ceiling_gripper_state_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::VacuumGripperState::ConstSharedPtr
       */
    void ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Assembly station AS1 states
       * @see as1_state_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AssemblyState::ConstSharedPtr
       */
    void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Assembly station AS2 states
       * @see as2_state_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AssemblyState::ConstSharedPtr
       */
    void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Assembly station AS3 states
       * @see as3_state_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AssemblyState::ConstSharedPtr
       */
    void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Assembly station AS4 states
       * @see as4_state_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AssemblyState::ConstSharedPtr
       */
    void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Kit Tray station 1 camera 
       * @see kts1_camera_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr
       */
    void kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Kit Tray station 2 camera
       * @see kts2_camera_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr
       */
    void kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Left bins station camera
       * @see left_bins_camera_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr
       */
    void left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Right bins station camera
       * @see right_bins_camera_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr
       */
    void right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Conveyor camera
       * @see conveyor_camera_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr
       */
    void conveyor_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to Conveyor camera and keeps a counter to the parts detected by the camera
       * @see conveyor_camera_counter_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr
       */
    void conveyor_camera_counter_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to first breakbeam on conveyor
       * @see breakbeam_start_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr
       */
    void breakbeam_start_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to second breakbeam on conveyor
       * @see breakbeam_end_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr
       */
    void breakbeam_end_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to keep count of parts crossing the first breakbeam on conveyor
       * @see breakbeam_start_counter_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr
       */
    void breakbeam_start_counter_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to keep count of parts crossing the second breakbeam on conveyor
       * @see breakbeam_end_counter_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr
       */
    void breakbeam_end_counter_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to get the current location of AGV1
       * @see agv1_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AGVStatus::ConstSharedPtr
       */
    void agv1_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to get the current location of AGV2
       * @see agv2_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AGVStatus::ConstSharedPtr
       */
    void agv2_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to get the current location of AGV3
       * @see agv3_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AGVStatus::ConstSharedPtr
       */
    void agv3_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    /**
       * @brief Subscriber Callback to get the current location of AGV4
       * @see agv4_cb()
       * @param msg a constant pointer of type ariac_msgs::msg::AGVStatus::ConstSharedPtr
       */
    void agv4_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg);
    /**
       * @brief To multiply two given poses
       * @see MultiplyPose()
       * @param p1 Pose 1 of type geometry_msgs::msg::Pose
       * @param p2 Pose 2 of type geometry_msgs::msg::Pose
       * @return the resulting pose of type geometry_msgs::msg::Pose
       */
    geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2);
    /**
       * @brief To build pose as geometry_msgs from given cartesian coordinates and orientation
       * @see BuildPose()
       * @param x x coordinate of type double
       * @param y y coordinate of type double
       * @param z z coordinate of type double
       * @param orientation orientation of type geometry_msgs::msg::Quaternion
       * @return the resulting pose of type geometry_msgs::msg::Pose
       */
    geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
    /**
       * @brief Returns pose in world frame for a given frame id.
       * @see FrameWorldPose()
       * @param frame_id frame id of type std::string
       * @return the resulting pose of type geometry_msgs::msg::Pose
       */
    geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
    /**
       * @brief Returns yaw for the given pose
       * @see GetYaw()
       * @param pose pose of type geometry_msgs::msg::Pose
       * @return the resulting pose of type geometry_msgs::msg::Pose
       */
    double GetYaw(geometry_msgs::msg::Pose pose);
    /**
       * @brief To calculate quaternion from given Euler rotations
       * @see QuaternionFromRPY()
       * @param r roll axis angle of type double
       * @param p pitch axis angle of type double
       * @param y yaw axis angle of type double
       * @return the resulting pose of type geometry_msgs::msg::Quaternion
       */
    geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);
    /**
       * @brief To calculate the orientation of robot given its orientation
       * @see SetRobotOrientation()
       * @param name name of the model of type std::string
       * @param mesh_file mesh file of the model of type std::string
       * @param model_pose pose of the model of type geometry_msgs::msg::Pose
       */
    geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock()); /*!< Buffer variable */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer); /*!< Transform listener*/

    // Trays
    std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_; /*!< Vector to store tray poses in kit tray 1 location */
    std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_; /*!< Vector to store tray poses in kit tray 2 location */

    // Bins
    std::vector<ariac_msgs::msg::PartPose> left_bins_parts_; /*!< Vector to store part poses in left bins */
    std::vector<ariac_msgs::msg::PartPose> right_bins_parts_; /*!< Vector to store part poses in right bins */
    std::vector<ariac_msgs::msg::PartPose> conv_part_; /*!< Vector to store part pose of part under conveyor camera*/
    std::vector<ariac_msgs::msg::PartPose> conv_part_current; /*!< vector to store current part pose on conveyor*/

    // MoveIt Interfaces
    moveit::planning_interface::MoveGroupInterface floor_robot_; /*!< MoveIt interface for floor robot*/
    moveit::planning_interface::MoveGroupInterface ceiling_robot_; /*!< MoveIt interface for ceiling robot*/
    moveit::planning_interface::PlanningSceneInterface planning_scene_; /*!< MoveIt planning scene*/

    trajectory_processing::TimeOptimalTrajectoryGeneration totg_; /*!< Used for trajectory generation*/

    // ARIAC Services
    rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;  /*!< Service to check faulty part*/
    rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_; /*!< Service to get pre assembly part poses*/
    rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_; /*!< Service to change floor robot gripper*/
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_; /*!< Service to enable/disable floor robot gripper*/
    rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_; /*!< Service to enable/disable ceiling robot gripper*/
 
    unsigned int competition_state_;/*!< Variable to store competition state */

    std::vector<OrderData> orders_; /*!< Variable to store normal orders */
    std::vector<OrderData> priority_orders_; /*!< Variable to store priority orders */
    std::vector<OrderData> incomplete_orders_; /*!< Variable to store incomplete orders */
    // Gripper State
    ariac_msgs::msg::VacuumGripperState floor_gripper_state_; /*!< Variable to store floor robot gripper state */
    ariac_msgs::msg::Part floor_robot_attached_part_; /*!< Variable to store floor robot part attached status */
    ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_; /*!< Variable to store ceiling robot gripper state */
    ariac_msgs::msg::Part ceiling_robot_attached_part_; /*!< Variable to store ceiling robot part attached status */

    bool faulty_part_discarded_flag{false};  /*!< Variable to store if a part was faulty and was discarded */

    int bin_read{0};/*!< To store bin topic read or not*/
    int conveyor_read{0};/*!< To store conveyor topic read or not*/
    int total_parts_for_conveyor{0};/*!< To store total parts to be spawned on conveyor*/
    int conv_part_counter_breakbeam1{0};/*!<Counter to store parts picked up from Location 1 on Conveyor*/
    
    //! Bin Parts map
    /*!
      Data Structure to store and update bin data
    */
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

    //! Conveyor Parts map
    /*!
      Data Structure to store and update conveyor data
    */
    std::map<uint8_t, std::map<uint8_t, uint8_t>>conveyor_dictionary = {
    {10, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {11, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {12, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}},
    {13, {{0, 0}, {1, 0},{2, 0}, {3, 0}, {4, 0}}}};
    //! Kitting Order map
    /*!
      Data structure to store kitting task plan // (type,color), bin 
    */
    std::map<uint8_t, std::pair<std::pair<uint8_t, uint8_t>, uint8_t>> kitting_part_details = {
    {1 , std::make_pair(std::make_pair(0, 0), 0)}, 
    {2 , std::make_pair(std::make_pair(0, 0), 0)}, 
    {3 , std::make_pair(std::make_pair(0, 0), 0)}, 
    {4 , std::make_pair(std::make_pair(0, 0), 0)}};
    //! Assembly Order map
    /*!
      Data Structure to store Assembly task
    */
    std::map<uint8_t, std::map<uint8_t, std::pair<uint8_t, uint8_t>>> assembly_part_details = {
    {1, {{1, std::make_pair(0, 0)}, {2, std::make_pair(0, 0)}, {3, std::make_pair(0, 0)}, {4, std::make_pair(0, 0)}}},
    {2, {{1, std::make_pair(0, 0)}, {2, std::make_pair(0, 0)}, {3, std::make_pair(0, 0)}, {4, std::make_pair(0, 0)}}},
    {3, {{1, std::make_pair(0, 0)}, {2, std::make_pair(0, 0)}, {3, std::make_pair(0, 0)}, {4, std::make_pair(0, 0)}}},
    {4, {{1, std::make_pair(0, 0)}, {2, std::make_pair(0, 0)}, {3, std::make_pair(0, 0)}, {4, std::make_pair(0, 0)}}}};
    //! Linear Rail positions
    /*!
      Linear Rail positions for major locations within ARIAC
    */
    std::map<std::string, double> rail_positions_ = {
        {"agv1", -4.5},
        {"agv2", -1.2},
        {"agv3", 1.2},
        {"agv4", 4.5},
        {"left_bins", 3}, 
        {"right_bins", -3}
    };
    //! Kit tray station 1 joint value targets 
    /*!
      Kit tray station 1 joint value targets for floor robot
    */
    std::map<std::string, double> floor_kts1_js_ = {
        {"linear_actuator_joint", 4.0},
        {"floor_shoulder_pan_joint", 1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}
    };
    //! Kit tray station 2 joint value targets 
    /*!
      Kit tray station 2 joint value targets for floor robot
    */
    std::map<std::string, double> floor_kts2_js_ = {
        {"linear_actuator_joint", -4.0},
        {"floor_shoulder_pan_joint", -1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}
    };
    //! Conveyor Pickup location 1 joint value targets 
    /*!
      Conveyor Pickup location 1 joint value targets for floor robot
    */
    std::map<std::string, double> floor_conveyor_parts_pickup_1 = {
        {"linear_actuator_joint", -0.5},
        {"floor_shoulder_pan_joint", 3.14},
        {"floor_shoulder_lift_joint", -1.0},
        {"floor_elbow_joint", 2.13},
        {"floor_wrist_1_joint", -2.76},
        {"floor_wrist_2_joint", -1.51},
        {"floor_wrist_3_joint", 0.0}
    };
    //! Conveyor Pickup location 2 joint value targets 
    /*!
      Conveyor Pickup location 2 joint value targets for floor robot
    */
    std::map<std::string, double> floor_conveyor_parts_pickup_2 = {
        {"linear_actuator_joint", 2.3},
        {"floor_shoulder_pan_joint", 3.14},
        {"floor_shoulder_lift_joint", -1.0},
        {"floor_elbow_joint", 2.13},
        {"floor_wrist_1_joint", -2.76},
        {"floor_wrist_2_joint", -1.51},
        {"floor_wrist_3_joint", 0.0}
    };
    //! Faulty part waste bins location joint value targets 
    /*!
      Waste bins location joint value targets for floor robot to drop faulty part
    */
    std::map<std::string, double> floor_waste_bin = {
        {"linear_actuator_joint", 0.0},
        {"floor_shoulder_pan_joint", 0},
        {"floor_shoulder_lift_joint", -0.88},
        {"floor_elbow_joint", 1.70},
        {"floor_wrist_1_joint", -2.4},
        {"floor_wrist_2_joint", -1.6},
        {"floor_wrist_3_joint", 0.0}
    };
    //! AS1 joint value targets Ceiling robot 
    /*!
      AS1 joint value targets Ceiling robot
    */
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
    //! AS2 joint value targets Ceiling robot 
    /*!
      AS2 joint value targets Ceiling robot
    */
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
    //! AS3 joint value targets Ceiling robot 
    /*!
      AS3 joint value targets Ceiling robot
    */
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
    //! AS4 joint value targets Ceiling robot 
    /*!
      AS4 joint value targets Ceiling robot
    */
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
    double kit_tray_thickness_ = 0.01; /*!< Variable to store tray thickness */
    double drop_height_ = 0.002; /*!< Variable to store drop heights for parts */
    double pick_offset_ = 0.003; /*!< Variable to store pick offset */
    double battery_grip_offset_ = -0.05; /*!< Variable to store battery pick offset for assembly operation */
    
    //! Map to store part types to string name
    /*!
      Map to store part types to string name
    */
    std::map<int, std::string> part_types_ = {
        {ariac_msgs::msg::Part::BATTERY, "battery"},
        {ariac_msgs::msg::Part::PUMP, "pump"},
        {ariac_msgs::msg::Part::REGULATOR, "regulator"},
        {ariac_msgs::msg::Part::SENSOR, "sensor"}
    };
    //! Map to store part colour to string name
    /*!
      Map to store part colour to string name
    */
    std::map<int, std::string> part_colors_ = {
        {ariac_msgs::msg::Part::RED, "red"},
        {ariac_msgs::msg::Part::BLUE, "blue"},
        {ariac_msgs::msg::Part::GREEN, "green"},
        {ariac_msgs::msg::Part::ORANGE, "orange"},
        {ariac_msgs::msg::Part::PURPLE, "purple"},
    };
    //! Map to store part heights
    /*!
      Map to store part heights
    */
    std::map<int, double> part_heights_ = {
        {ariac_msgs::msg::Part::BATTERY, 0.04},
        {ariac_msgs::msg::Part::PUMP, 0.12},
        {ariac_msgs::msg::Part::REGULATOR, 0.07},
        {ariac_msgs::msg::Part::SENSOR, 0.07}
    };
    //! Map to store quadrant offset on Kit trays
    /*!
      Map to store quadrant offset on Kit trays
    */
    std::map<int, std::pair<double, double>> quad_offsets_ = {
        {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
        {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
    };
    
    //! Array to store available locations on right bins number 1 and 2
    /*!
      Map to store quadrant offset on Kit trays
    */
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
    
    //! Array to store available locations on right bins number 1 and 2
    /*!
      Map to store quadrant offset on Kit trays
    */
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
   
   //! Array to store available agvs
    /*!
      Array to store non occupied AGVs
    */
   std::array<bool, 4> agv_availablity{{true, true, true, true}}; 
}; 