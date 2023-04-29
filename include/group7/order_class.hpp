/**
 *  @brief     This file contains the class definition of Order Class.
 *  @author    Ishan Tamrakar
 *  @author    Krishna Hundekari
 *  @author    Pranav Shinde
 *  @author    Vineet Singh
 *  @version   1.0
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
    std::uint8_t color; /** the color of the part*/
    std::uint8_t type; //! the type of the part
    std::int32_t header_stamp_sec; // !the seconds component of the timestamp 
    std::int32_t header_stamp_nanos; /*!< the nanoseconds component of the timestamp */
    std::string frame_id; //<! the reference frame ID for the assembled pose of the part
    _Float64 position_x; // the x-coordinate of the position of the part
    _Float64 position_y; // the y-coordinate of the position of the part
    _Float64 position_z; // the z-coordinate of the position of the part
    _Float64 orientation_x; // the x-coordinate of the orientation of the part
    _Float64 orientation_y; // the y-coordinate of the orientation of the part
    _Float64 orientation_z; // the z-coordinate of the orientation of the part
    _Float64 orientation_w; // the w-coordinate of the orientation of the part
    _Float64 install_direction_x; // the x-coordinate of the installation direction of the part
    _Float64 install_direction_y; // the y-coordinate of the installation direction of the part
    _Float64 install_direction_z; // the z-coordinate of the installation direction of the part

    // default constructor that initializes all member variables to 0 or an empty string                                                        
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

// a class that represents information about a part used in an assembly task
/**
 * @brief class that represents information about a part used in an assembly task
 * 
 */
class AssemblyPartInfo
{
public:
    std::uint8_t color; // the color of the part
    std::uint8_t type; // the type of the part
    std::int32_t header_stamp_sec; // the seconds component of the timestamp 
    std::int32_t header_stamp_nanos; // the nanoseconds component of the timestamp 
    std::string frame_id; // the reference frame ID for the assembled pose of the part
    _Float64 position_x; // the x-coordinate of the position of the part
    _Float64 position_y; // the y-coordinate of the position of the part
    _Float64 position_z; // the z-coordinate of the position of the part
    _Float64 orientation_x; // the x-coordinate of the orientation of the part
    _Float64 orientation_y; // the y-coordinate of the orientation of the part
    _Float64 orientation_z; // the z-coordinate of the orientation of the part
    _Float64 orientation_w; // the w-coordinate of the orientation of the part
    _Float64 install_direction_x; // the x-coordinate of the installation direction of the part
    _Float64 install_direction_y; // the y-coordinate of the installation direction of the part
    _Float64 install_direction_z; // the z-coordinate of the installation direction of the part 

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

// 
/**
 * @brief a class that represents information about a part used in an kiting task
 * 
 */
class KittingPartInfo
{
public:
    std::uint8_t color;
    std::uint8_t type;
    std::uint8_t quadrant;

    // default constructor that initializes all member variables to 0 or an empty string 
    KittingPartInfo() : color(0), type(0), quadrant(0) {}

    // Constructor that initializes the member variables based on the order message 
    // and the index of the part in the kiiting task
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
    int number_of_parts; // The number of assembly parts in the order
    std::vector<CombinedPartInfo> parts_; // A vector containing information about each assembly part
    CombinedPartInfo combined_part; // A default instance ofAssemblyPartInfo class

    

    /**
     * @brief Default constructor
     * 
     */
    CombinedPartsInfo() : number_of_parts(0), parts_() {}

    // 
    /**
     * @brief Constructor with the shared pointer to the order message as input
     * 
     * //!@param msg 
     */
    CombinedPartsInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        // Get the vector of parts from the order message
        std::vector<ariac_msgs::msg::AssemblyPart> parts = msg->combined_task.parts;
        // Set the number of parts to the size of the vector
        this->number_of_parts = parts.size();
        // If there are no parts in the vector, create a combined part with default values and add it to the vector
        if (msg->combined_task.parts.empty())
        {
            this->parts_.push_back(combined_part);
        }
        // If there are parts in the vector, loop over them and create a combined part info for each
        else
        {
            for (int i = 0; i < number_of_parts; i++)
            {
                // Create a new combined part info for the current part and add it to the vector of part infos
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
    int number_of_parts; // The number of assembly parts in the order
    std::vector<AssemblyPartInfo> parts_; // A vector containing information about each assembly part
    AssemblyPartInfo assembly_part; // A default instance ofAssemblyPartInfo class
    
    /**
     * @brief Default constructor initializes number_of_parts to 0 and creates an empty vector of AssemblyPartInfo objects.
     * 
     */
    AssemblyPartsInfo() : number_of_parts(0), parts_() {}

    // 
    /**
     * @brief Constructor that takes an Order message pointer as an argument and initializes AssemblyPartsInfo using the information from the message.
     * 
     * @param msg Order message pointer
     */
    AssemblyPartsInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        // Get the vector of assembly task parts from the message.
        std::vector<ariac_msgs::msg::AssemblyPart> parts = msg->assembly_task.parts;

        // Set the number of parts to the size of the vector.
        this->number_of_parts = parts.size();

        // If the vector of parts is empty, add an empty AssemblyPartInfo object to the vector.
        if (msg->assembly_task.parts.empty())
        {
            this->parts_.push_back(assembly_part);
        }
        // Otherwise, create an AssemblyPartInfo object for each part in the vector 
        //and add it to the vector of AssemblyPartInfo objects.
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

    int number_of_parts; // The number of kitting parts in the order
    std::vector<KittingPartInfo> parts_; // A vector containing information about each kitting part
    KittingPartInfo kitting_part; // A default instance of KittingPartInfo class


    // Default constructor
    KittingPartsInfo() : number_of_parts(0), parts_() {}

    // Constructor that initializes the object using an order message pointer
    KittingPartsInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        std::vector<ariac_msgs::msg::KittingPart> parts = msg->kitting_task.parts;
        this->number_of_parts = parts.size();

        // If there are no kitting parts in the order, add a default KittingPartInfo object to the parts_ vector
        if (msg->kitting_task.parts.empty())
        {
            this->parts_.push_back(kitting_part);
        }
        else
        {
             // Iterate through the kitting parts and add their information to the parts_ vector
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
    std::uint8_t station; // station number where the task should be performed
    CombinedPartsInfo combined_parts; // information about the parts needed for the combined task

    // Constructor for CombinedInfo class
    CombinedInfo() : station(0) {}

    // Constructor that initializes the object using an Order message
    CombinedInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        // set the station number
        this->station = msg->combined_task.station;
         // set the information about the parts for the task
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
    std::vector<std::uint8_t> agv_numbers;// Vector to store AGV numbers
    std::uint8_t station; // Station number
    AssemblyPartsInfo assembly_parts;// Object of class AssemblyPartsInfo to store parts 

    // Constructor for AssemblyInfo class
    AssemblyInfo() : agv_numbers(0), station(0) {}

    // Constructor for AssemblyInfo class that takes in an Order message as input
    AssemblyInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {   
        // Store the AGV numbers and station number from the Order message
        this->agv_numbers = msg->assembly_task.agv_numbers;
        this->station = msg->assembly_task.station;
            
        // Create an AssemblyPartsInfo object to store information about the assembly parts
        // and store it in the assembly_parts variable
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

    std::uint8_t agv_number; // the AGV number where kitting should be done
    int8_t tray_id; // the tray id for the kitting task
    std::uint8_t destination; // the destination where the kitting should be done
    KittingPartsInfo kitting_parts; // the kitting parts info
                   
    // Constructor for KittingInfo class
    KittingInfo() : agv_number(0), tray_id(0), destination(0) {}

    // Constructor that sets kitting part information from an Order message
    KittingInfo(const ariac_msgs::msg ::Order::SharedPtr msg)
    {
        this->agv_number = msg->kitting_task.agv_number;
        this->tray_id = msg->kitting_task.tray_id;
        this->destination = msg->kitting_task.destination;
        this->kitting_parts = KittingPartsInfo(msg);
    }
};
/**
 * @brief This is the Class Definition for OrderData
 * 
 */
class OrderData
{
public:
    // Member variables
    std::string id; // Order ID
    int type; // Type of order (0 = Kitting, 1 = Assembly, 2 = Combined)
    bool priority; // Priority of the order (true = high priority, false = low priority)
    KittingInfo kitting; // Information about the kitting task (if received)
    AssemblyInfo assembly; // Information about the assembly task (if received)
    CombinedInfo combined; // Information about the combined task (if received)

    /**
     * @brief Constructor to initialise the class attributes
     * 
     * @param msg 
     */
    OrderData(const std::shared_ptr<ariac_msgs::msg ::Order> msg)
    {
        this->id = msg->id;   // Set order ID
        this->type = msg->type; // Set order type
        this->priority = msg->priority; // Set order priority
        this->kitting = KittingInfo(msg);   //! Create KittingInfo object from message and assign to kitting member variable
        this->assembly = AssemblyInfo(msg);  // Create AssemblyInfo object from message and assign to assembly member variable
        this->combined = CombinedInfo(msg);  // Create CombinedInfo object from message and assign to combined member variable
    
    }
};