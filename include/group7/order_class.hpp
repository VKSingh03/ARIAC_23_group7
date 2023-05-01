/**
 *  @brief     This file contains the class definition of Order Class.
 *  @author    Vineet Singh
 *  @author    Krishna Hundekari
 *  @author    Ishan Tamrakar
 *  @author    Pranav Shinde
 *  @version   4.0
 */
#pragma once

#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/srv/submit_order.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <stdint.h>
/**
 * @brief This Class includes the information of part for combined task
 *  
 * 
 */
class CombinedPartInfo
{
public:

    /**
     * @brief The color of the part.
     */
    std::uint8_t color;

    /**
     * @brief The type of the part.
     *       
     */
    std::uint8_t type;

    /**
     * @brief The seconds component of the timestamp.
     */
    std::int32_t header_stamp_sec;

    /**
     * @brief The nanoseconds component of the timestamp.
     *   
     */
    std::int32_t header_stamp_nanos;

    /**
     * @brief The reference frame ID for the assembled pose of the part.
     */
    std::string frame_id;

    /**
     * @brief The x-coordinate of the position of the part.
     */
    _Float64 position_x;

    /**
     * @brief The y-coordinate of the position of the part.
     */
    _Float64 position_y;

    /**
     * @brief The z-coordinate of the position of the part.
     */
    _Float64 position_z;

    /**
     * @brief The x-coordinate of the orientation of the part.
     */
    _Float64 orientation_x;

    /**
     * @brief The y-coordinate of the orientation of the part.
     */
    _Float64 orientation_y;

    /**
     * @brief The z-coordinate of the orientation of the part.
     */
    _Float64 orientation_z;

    /**
     * @brief The w-coordinate of the orientation of the part.
     */
    _Float64 orientation_w;

    /**
     * @brief The x-coordinate of the installation direction of the part.
     */
    _Float64 install_direction_x;

    /**
     * @brief The y-coordinate of the installation direction of the part.
     */
    _Float64 install_direction_y;

    /**
     * @brief The z-coordinate of the installation direction of the part.
     */
    _Float64 install_direction_z;

    /**
     * @brief Constructor for a new Combined Part Info object.
     * Default constructor that initializes all member variables to 0 or an empty string 
     */
    CombinedPartInfo() : color(0), type(0), header_stamp_sec(0), header_stamp_nanos(0), frame_id(""), position_x(0), 
    position_y(0), position_z(0), orientation_x(0), orientation_y(0), orientation_z(0), orientation_w(0), 
    install_direction_x(0), install_direction_y(0), install_direction_z(0) {}

    // 
    /**
     * @brief Constructor that initializes the member variables based on the order message and the index of the part in the combined task
     * 
     * @param which_part 
     * @param msg 
     */
    CombinedPartInfo(int which_part, const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        // initialize the variables based on the information in the order message
        this->color = msg->combined_task.parts[which_part].part.color;
        this->type = msg->combined_task.parts[which_part].part.type;
        this->header_stamp_sec = msg->combined_task.parts[which_part].assembled_pose.header.stamp.sec;
        this->header_stamp_nanos = msg->combined_task.parts[which_part].assembled_pose.header.stamp.nanosec;
        this->frame_id = msg->combined_task.parts[which_part].assembled_pose.header.frame_id;
        this->position_x = msg->combined_task.parts[which_part].assembled_pose.pose.position.x;
        this->position_y = msg->combined_task.parts[which_part].assembled_pose.pose.position.y;
        this->position_z = msg->combined_task.parts[which_part].assembled_pose.pose.position.z;
        this->orientation_x = msg->combined_task.parts[which_part].assembled_pose.pose.orientation.x;
        this->orientation_y = msg->combined_task.parts[which_part].assembled_pose.pose.orientation.y;
        this->orientation_z = msg->combined_task.parts[which_part].assembled_pose.pose.orientation.z;
        this->orientation_w = msg->combined_task.parts[which_part].assembled_pose.pose.orientation.w;
        this->install_direction_x = msg->combined_task.parts[which_part].install_direction.x;
        this->install_direction_y = msg->combined_task.parts[which_part].install_direction.y;
        this->install_direction_z = msg->combined_task.parts[which_part].install_direction.z;
    }
};

/**
 * @brief class that represents information about a part used in an assembly task
 * 
 */
class AssemblyPartInfo
{
public:
    /**
     * @brief The color of the part.
     */
    std::uint8_t color;

    /**
     * @brief The type of the part.
     */
    std::uint8_t type;

    /**
     * @brief The seconds component of the timestamp.
     */
    std::int32_t header_stamp_sec;

    /**
     * @brief The nanoseconds component of the timestamp.
     */
    std::int32_t header_stamp_nanos;

    /**
     * @brief The reference frame ID for the assembled pose of the part.
     */
    std::string frame_id;

    /**
     * @brief The x-coordinate of the position of the part.
     */
    _Float64 position_x;

    /**
     * @brief The y-coordinate of the position of the part.
     */
    _Float64 position_y;

    /**
     * @brief The z-coordinate of the position of the part.
     */
    _Float64 position_z;

    /**
     * @brief The x-coordinate of the orientation of the part.
     */
    _Float64 orientation_x;

    /**
     * @brief The y-coordinate of the orientation of the part.
     */
    _Float64 orientation_y;

    /**
     * @brief The z-coordinate of the orientation of the part.
     */
    _Float64 orientation_z;

    /**
     * @brief The w-coordinate of the orientation of the part.
     */
    _Float64 orientation_w;

    /**
     * @brief The x-coordinate of the installation direction of the part.
     */
    _Float64 install_direction_x;

    /**
     * @brief The y-coordinate of the installation direction of the part.
     */
    _Float64 install_direction_y;

    /**
     * @brief The z-coordinate of the installation direction of the part.
     */
    _Float64 install_direction_z;
   /**
    * @brief default constructor that initializes all member variables to 0 or an empty string 
    * 
    */
    AssemblyPartInfo() : color(0), type(0), header_stamp_sec(0), header_stamp_nanos(0), frame_id(""), 
    position_x(0), position_y(0), position_z(0), orientation_x(0), orientation_y(0), orientation_z(0), 
    orientation_w(0), install_direction_x(0), install_direction_y(0), install_direction_z(0) {}

    // 
    /**
     * @brief constructor that initializes the member variables based on the order message and the index of the part in the assembly task
     * 
     * @param which_part 
     * @param msg 
     */
    AssemblyPartInfo(int which_part, const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        // initialize the variables based on the information in the order message
        this->color = msg->assembly_task.parts[which_part].part.color;
        this->type = msg->assembly_task.parts[which_part].part.type;
        this->header_stamp_sec = msg->assembly_task.parts[which_part].assembled_pose.header.stamp.sec;
        this->header_stamp_nanos = msg->assembly_task.parts[which_part].assembled_pose.header.stamp.nanosec;
        this->frame_id = msg->assembly_task.parts[which_part].assembled_pose.header.frame_id;
        this->position_x = msg->assembly_task.parts[which_part].assembled_pose.pose.position.x;
        this->position_y = msg->assembly_task.parts[which_part].assembled_pose.pose.position.y;
        this->position_z = msg->assembly_task.parts[which_part].assembled_pose.pose.position.z;
        this->orientation_x = msg->assembly_task.parts[which_part].assembled_pose.pose.orientation.x;
        this->orientation_y = msg->assembly_task.parts[which_part].assembled_pose.pose.orientation.y;
        this->orientation_z = msg->assembly_task.parts[which_part].assembled_pose.pose.orientation.z;
        this->orientation_w = msg->assembly_task.parts[which_part].assembled_pose.pose.orientation.w;
        this->install_direction_x = msg->assembly_task.parts[which_part].install_direction.x;
        this->install_direction_y = msg->assembly_task.parts[which_part].install_direction.y;
        this->install_direction_z = msg->assembly_task.parts[which_part].install_direction.z;
    }
};

/**
 * @brief a class that represents information about a part used in an kiting task
 * 
 */
class KittingPartInfo
{
public:
    /**
     * @brief The color of the part.
     */
    std::uint8_t color;
      /**
     * @brief The type of the part.
     */
    std::uint8_t type;
    
      /**
     * @brief The qaudrant of the tray where the part will be placedt.
     */

    std::uint8_t quadrant;

    /**
    * @brief Default constructor that initializes all member variables to 0 or an empty string 
    * 
    */
    KittingPartInfo() : color(0), type(0), quadrant(0) {}

    /**
    * @brief  Constructor that initializes the member variables based on the order message  
    *  and the index of the part in the kiiting task
    */

    KittingPartInfo(int which_part, const ariac_msgs::msg::Order::SharedPtr msg)
    {
        // initialize the variables based on the information in the order message
        this->color = msg->kitting_task.parts[which_part].part.color;
        this->type = msg->kitting_task.parts[which_part].part.type;
        this->quadrant = msg->kitting_task.parts[which_part].quadrant;
    }
};

/**
 * @brief This class defines the information about the Combined parts in an order.
 * 
 */
class CombinedPartsInfo
{
public:

    /**
     * @brief The number of assembly parts in the order.
     */
    int number_of_parts;

    /**
     * @brief A vector containing information about each assembly part.
     */
    std::vector<CombinedPartInfo> parts_;

    /**
     * @brief A default instance of CombinedPartInfo class.
     */
    CombinedPartInfo combined_part;
   
    /**
     * @brief Default constructor
     * 
     */
    CombinedPartsInfo() : number_of_parts(0), parts_() {}

    /**
     * @brief Constructor with the shared pointer to the order message as input
     * 
     * //!@param msg 
     */
    CombinedPartsInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        std::vector<ariac_msgs::msg::AssemblyPart> parts = msg->combined_task.parts;
        this->number_of_parts = parts.size();
        if (msg->combined_task.parts.empty())
        {
            this->parts_.push_back(combined_part);
        }
        else
        {
            for (int i = 0; i < number_of_parts; i++)
            {
                this->combined_part = CombinedPartInfo(i, msg);
                this->parts_.push_back(combined_part);
            }
        }
    }
};

/**
 * @brief This class defines the information about the assembly parts in an order.
 * 
 */
class AssemblyPartsInfo
{
public:
    /**
     * @brief The number of assembly parts in the order.
     */
    int number_of_parts;

    /**
     * @brief A vector containing information about each assembly part.
     */
    std::vector<AssemblyPartInfo> parts_;

    /**
     * @brief A default instance of AssemblyPartInfo class.
     */
    AssemblyPartInfo assembly_part;

    
    /**
     * @brief Default constructor initializes number_of_parts to 0 and creates an empty vector of AssemblyPartInfo objects.
     * 
     */
    AssemblyPartsInfo() : number_of_parts(0), parts_() {}

    /**
     * @brief Constructor that takes an Order message pointer as an argument and initializes AssemblyPartsInfo using the information from the message.
     * 
     * @param msg Order message pointer
     */
    AssemblyPartsInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        std::vector<ariac_msgs::msg::AssemblyPart> parts = msg->assembly_task.parts;
        this->number_of_parts = parts.size();
        if (msg->assembly_task.parts.empty())
        {
            this->parts_.push_back(assembly_part);
        }
        else
        {
            for (int i = 0; i < number_of_parts; i++)
            {
                this->assembly_part = AssemblyPartInfo(i, msg);
                this->parts_.push_back(assembly_part);
            }
        }
    }
};

/**
 * @brief This class defines the information about the kitting parts in an order.
 * 
 */
class KittingPartsInfo
{
public:

    /**
     * @brief The number of kitting parts in the order.
     */
    int number_of_parts;

    /**
     * @brief A vector containing information about each kitting part.
     */
    std::vector<KittingPartInfo> parts_;

    /**
     * @brief A default instance of KittingPartInfo class.
     */
    KittingPartInfo kitting_part;

    /**
     * @brief Default constructor initializes number_of_parts to 0 and creates an empty vector of KittingPartInfo objects.
     * 
     */
    KittingPartsInfo() : number_of_parts(0), parts_() {}

    /**
     * @brief Constructor that takes an Order message pointer as an argument and initializes AssemblyPartsInfo using the information from the message.
     * 
     * @param msg Order message pointer
     */
    KittingPartsInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        std::vector<ariac_msgs::msg::KittingPart> parts = msg->kitting_task.parts;
        this->number_of_parts = parts.size();
        if (msg->kitting_task.parts.empty())
        {
            this->parts_.push_back(kitting_part);
        }
        else
        {
            for (int i = 0; i < this->number_of_parts; i++)
            {
                this->kitting_part = KittingPartInfo(i, msg);
                this->parts_.push_back(kitting_part);
            }
        }
    }
};

/**
 * @brief Class represents combined task information in an order
 * 
 */
class CombinedInfo
{
public:
    /**
     * @brief Station number where the task should be performed.
     */
    std::uint8_t station;

    /**
     * @brief Information about the parts needed for the combined task.
     */
    CombinedPartsInfo combined_parts;

    /**
     * @brief Default constructor for CombinedInfo class that initializes station number to 0.
     * 
     */
    CombinedInfo() : station(0) {}

    /**
     * @brief Default constructor for CombinedInfo class that initializes the object using an Order message
     * 
     */
    CombinedInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        this->station = msg->combined_task.station;
        this->combined_parts = CombinedPartsInfo(msg);
    }
};

/**
 * @brief Class represents assembly information in an order
 * 
 */
class AssemblyInfo
{
public:

    /**
     * @brief Vector to store AGV numbers.
     */
    std::vector<std::uint8_t> agv_numbers;

    /**
     * @brief Station number.
     */
    std::uint8_t station;

    /**
     * @brief Object of class AssemblyPartsInfo to store parts.
     */
    AssemblyPartsInfo assembly_parts;
    /**
     * @brief Constructor for AssemblyInfo class
     *
     * Initializes the attributes of the class to default values.
     */
    AssemblyInfo() : agv_numbers(0), station(0) {}

    /**
     * @brief Constructor for AssemblyInfo class that takes in an Order message as input
     *
     * @param msg a shared pointer to an Order message object
     */
    AssemblyInfo(const ariac_msgs::msg::Order::SharedPtr msg)
    {   
        this->agv_numbers = msg->assembly_task.agv_numbers;
        this->station = msg->assembly_task.station;
        this->assembly_parts = AssemblyPartsInfo(msg);
    }
};


/**
 * @brief Class represents kitting information in an order
 * 
 */
class KittingInfo
{
public:

    /**
     * @brief The AGV number where the kitting should be done
     */
    std::uint8_t agv_number;

    /**
     * @brief The tray ID for the kitting task
     */
    int8_t tray_id;

    /**
     * @brief The destination where the kitting should be done
     */
    std::uint8_t destination;

    /**
     * @brief Information about the kitting parts required for the task
     */
    KittingPartsInfo kitting_parts;

    /**
     * @brief Default constructor for KittingInfo class
     * 
     */
    KittingInfo() : agv_number(0), tray_id(0), destination(0) {}

    /**
     * @brief Constructor for KittingInfo class that sets kitting part information from an Order message
     * 
     * @param msg A shared pointer to the Order message containing kitting task information
     */
    KittingInfo(const ariac_msgs::msg::Order::SharedPtr msg)
    {
        this->agv_number = msg->kitting_task.agv_number;
        this->tray_id = msg->kitting_task.tray_id;
        this->destination = msg->kitting_task.destination;
        this->kitting_parts = KittingPartsInfo(msg);
    }
};
/**
 * @brief Class to store the Order received from ARIAC
 * 
 */
/**
 * @brief This is the Class Definition for OrderData
 * 
 */
class OrderData
{
public:

    /**
     * @brief The unique ID of the order.
     */
    std::string id;

    /**
     * @brief The type of the order.
     * 0 - Kitting
     * 1 - Assembly
     * 2 - Combined
     */
    int type;

    /**
     * @brief The priority of the order.
     * true - high priority
     * false - low priority
     */
    bool priority;

    /**
     * @brief Information about the kitting task (if received).
     */
    KittingInfo kitting;

    /**
     * @brief Information about the assembly task (if received).
     */
    AssemblyInfo assembly;

    /**
     * @brief Information about the combined task (if received).
     */
    CombinedInfo combined;


    /**
     * @brief A flag indicating whether the task is of priority or not.
     */
    bool importance_flag{false};

    /**
     * @brief A flag indicating whether the task has been left incomplete to complete a task of higher priority.
     */
    bool abandoned_{false};

    /**
     * @brief A flag indicating whether the positions of parts in an AGV have been extracted in the incomplete order.
     */
    bool agv_part_poses_extracted{false};

    /**
     * @brief A flag indicating whether a combined tray has been placed on an AGV in the incomplete order.
     */
    bool combined_tray_placed_on_agv{false};

    /**
     * @brief The ID of the selected AGV for a combined tray.
     */
    int combined_agv_selected = -1;

    /**
     * @brief The ID of the selected combined tray.
     */
    int combined_tray_selected = 0;

    /**
     * @brief A map containing the details of the kitting parts already placed on the tray before the task was abandoned to take
     * new priority order. This map saves the progress of the kitting task.
     */
    std::map<uint8_t, std::pair<std::pair<uint8_t, uint8_t>, uint8_t>> kitting_part_details_orderdata = {
        {1 , std::make_pair(std::make_pair(0, 0), 0)}, 
        {2 , std::make_pair(std::make_pair(0, 0), 0)}, 
        {3 , std::make_pair(std::make_pair(0, 0), 0)}, 
        {4 , std::make_pair(std::make_pair(0, 0), 0)}
        };

    /**
     * @brief An array indicating the status of assembly for each part that has been assebled before the assebly task is abandoned to take new 
     * priority order. This array saves the progress of the assembly task.
     */
    std::array<int, 4> status_of_assembly{{1,1,1,1}};

    /**
     * @brief Constructor to initialise the class attributes
     * 
     * @param msg 
     */
    OrderData(const std::shared_ptr<ariac_msgs::msg ::Order> msg)
    {
        this->id = msg->id; 
        this->type = msg->type; 
        this->priority = msg->priority;
        this->kitting = KittingInfo(msg);
        this->assembly = AssemblyInfo(msg); 
        this->combined = CombinedInfo(msg);
    
    }
};