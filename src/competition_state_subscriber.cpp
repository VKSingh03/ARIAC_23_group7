# include "competition_state_subscriber.hpp"
# include "order_class.hpp"
# include <std_srvs/srv/trigger.hpp>
# include <memory>
# include <vector>
# include <iostream>  
#include <fstream>

std::string CompetitorControlSystem::DestinationtoString(uint8_t dest){
    if (dest == ariac_msgs::msg::KittingTask::ASSEMBLY_FRONT)
        return "Assembly Front";
    else if (dest == ariac_msgs::msg::KittingTask::ASSEMBLY_BACK)
        return "Assembly Back";
    else if (dest == ariac_msgs::msg::KittingTask::WAREHOUSE)
        return "Warehouse";
    else
        return "unknown";
}

std::string CompetitorControlSystem::PartTypetoString(uint8_t part_type){
    if (part_type == ariac_msgs::msg::Part::BATTERY)
        return "battery";
    else if (part_type == ariac_msgs::msg::Part::PUMP)
        return "pump";
    else if (part_type == ariac_msgs::msg::Part::REGULATOR)
        return "regulator";
    else if (part_type == ariac_msgs::msg::Part::SENSOR)
        return "sensor";
    else
        return "unknown";
}

std::string CompetitorControlSystem::PartColortoString(uint8_t part_color){
    if (part_color == ariac_msgs::msg::Part::RED)
        return "red";
    else if (part_color == ariac_msgs::msg::Part::GREEN)
        return "green";
    else if (part_color == ariac_msgs::msg::Part::BLUE)
        return "blue";
    else if (part_color == ariac_msgs::msg::Part::PURPLE)
        return "purple";
    else if (part_color == ariac_msgs::msg::Part::ORANGE)
        return "orange";
    else
        return "unknown";
}

std::string CompetitorControlSystem::StationtoString(uint8_t station){
    if (station == ariac_msgs::msg::CombinedTask::AS1)
        return "AS1";
    else if (station == ariac_msgs::msg::CombinedTask::AS2)
        return "AS2";
    else if (station == ariac_msgs::msg::CombinedTask::AS3)
        return "AS3";
    else if (station == ariac_msgs::msg::CombinedTask::AS4)
        return "AS4";
    else
        return "unknown";
}

geometry_msgs::msg::Quaternion CompetitorControlSystem::QuaternionFromRPY(double r, double p, double y){
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}

void CompetitorControlSystem::agv1_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg){
    agv1_location = msg->location; 
}

void CompetitorControlSystem::agv2_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg){
    agv2_location = msg->location; 
}

void CompetitorControlSystem::agv3_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg){
    agv3_location = msg->location; 
}

void CompetitorControlSystem::agv4_cb(const ariac_msgs::msg::AGVStatus::ConstSharedPtr msg){
    agv4_location = msg->location; 
}

void CompetitorControlSystem::as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS1, *msg);
}

void CompetitorControlSystem::as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS2, *msg);
}

void CompetitorControlSystem::as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS3, *msg);
}

void CompetitorControlSystem::as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS4, *msg);
}

void CompetitorControlSystem::kts1_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg){
    if (!kts1_camera_recieved_data) {
        RCLCPP_INFO(this->get_logger(), "Received data from kts1 camera");
        kts1_camera_recieved_data = true;
    }
    
    kts1_trays_ = msg->tray_poses;
    kts1_camera_pose_ = msg->sensor_pose;
}

void CompetitorControlSystem::kts2_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg){
    if (!kts2_camera_recieved_data) {
        RCLCPP_INFO(this->get_logger(), "Received data from kts2 camera");
        kts2_camera_recieved_data = true;
    }
    kts2_trays_ = msg->tray_poses;
    kts2_camera_pose_ = msg->sensor_pose;
}

void CompetitorControlSystem::left_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!left_bins_camera_recieved_data) {
    RCLCPP_INFO(this->get_logger(), "Received data from left bins camera");
    left_bins_camera_recieved_data = true;
    // The commented section the poses as a text file for later use. 
    // std::ofstream outfile("left_bins.txt");
    // for(auto i:msg->part_poses){
    //     outfile << i.pose.position.x <<", "<< i.pose.position.y<<", "<<i.pose.position.z << std::endl;
    // }
    // outfile.close();
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;
}

void CompetitorControlSystem::right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!right_bins_camera_recieved_data) {
    RCLCPP_INFO(this->get_logger(), "Received data from right bins camera");
    right_bins_camera_recieved_data = true;
    // The commented section the poses as a text file for later use. 
    // std::ofstream outfile("right_bins.txt");
    // for(auto i:msg->part_poses){
    //     outfile << i.pose.position.x <<", "<< i.pose.position.y <<", "<<i.pose.position.z <<std::endl;
    // }
    // outfile.close();
    // for(auto i: msg->part_poses)
    //     RCLCPP_INFO_STREAM(get_logger()," X: '"<< (std::to_string(part_bin_location))<<"'");
  }

  right_bins_parts_ = msg->part_poses;
  right_bins_camera_pose_ = msg->sensor_pose;
}

void CompetitorControlSystem::conveyor_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg){
    if (!conveyor_camera_received_data && (msg->part_poses.size() != 0)) {
        RCLCPP_INFO(this->get_logger(), "Received data from conveyor camera");
        conveyor_camera_received_data = true;
    }
}

void CompetitorControlSystem::conveyor_camera_counter_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg){
    if (!detected_conveyor_camera && (msg->part_poses.size() != 0)) {
        // RCLCPP_INFO(this->get_logger(), "Added part counter for conveyor ");
        detected_conveyor_camera = true;
        conv_part_.push_back(msg->part_poses[0]); 
        // RCLCPP_INFO_STREAM(get_logger()," Detected part:  '"<< (std::to_string(part[0].part.type))<<"'");
    }
    else if(detected_conveyor_camera && (msg->part_poses.size() < 1)){
        detected_conveyor_camera = false;
    }
    conv_camera_pose_ = msg->sensor_pose; 
}

void CompetitorControlSystem::breakbeam_start_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if(!breakbeam_start_received_data && msg->object_detected == true){
        breakbeam_start_received_data = true;
    }
}

void CompetitorControlSystem::breakbeam_end_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if(!breakbeam_end_received_data && msg->object_detected == true){
        breakbeam_end_received_data = true;
    }
}

void CompetitorControlSystem::breakbeam_start_counter_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if((msg->object_detected == true) && (detected_first_breakbeam == false)){
        breakbeam_one_counter += 1;
        detected_first_breakbeam = true; 
        // RCLCPP_INFO_STREAM(get_logger()," Breakbeam 1 counter '"<< (std::to_string(breakbeam_one_counter))<<"'");
    }
    else if ((msg->object_detected == false) && (detected_first_breakbeam == true)){
        detected_first_breakbeam = false; 
    }
}

void CompetitorControlSystem::breakbeam_end_counter_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if( (msg->object_detected == true) && (detected_second_breakbeam == false)){
        breakbeam_two_counter +=1;
        detected_second_breakbeam = true; 
        // RCLCPP_INFO_STREAM(get_logger()," Breakbeam 2 counter '"<< (std::to_string(breakbeam_two_counter))<<"'");
    }
    else if ((msg->object_detected == false) && (detected_second_breakbeam == true)){
        detected_second_breakbeam = false; 
    }
}

void CompetitorControlSystem::competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
    
    competition_state_ = msg->competition_state;
}

void CompetitorControlSystem::order_callback(const ariac_msgs::msg::Order::SharedPtr msg){   
    RCLCPP_INFO_STREAM(this->get_logger(), " Received order Id: " << msg->id);
    // Creating object of order_data to store current order
    OrderData order(msg); 
    //Storing order based on priority
    if(order.priority == true){
        priority_orders_.push_back(order);
        RCLCPP_INFO(this->get_logger()," Added priority order '%s' to open orders", order.id.c_str());
    }
    else{
        orders_.push_back(order);
        RCLCPP_INFO(this->get_logger()," Added normal order '%s' to open orders", order.id.c_str());
    }
}

int CompetitorControlSystem::bin_get_quantity_for_this_part(uint8_t bin, uint8_t part, uint8_t color){
    if (bin_dictionary.count(bin) == 0 || bin_dictionary[bin].count(part) == 0 || bin_dictionary[bin][part].count(color) == 0) {
        throw std::out_of_range("One of the keys does not exist in the map");
    }
    return bin_dictionary[bin][part][color];
}

void CompetitorControlSystem::bin_set_quantity_for_this_part(uint8_t bin, uint8_t part, uint8_t color, int value) {
    bin_dictionary[bin][part][color] += value;
}

void CompetitorControlSystem::conveyor_set_quantity_for_this_part(uint8_t part, uint8_t color, int value) {
    conveyor_dictionary[part][color] += value;
}

void CompetitorControlSystem::bin_status_callback(const ariac_msgs::msg::BinParts::ConstSharedPtr msg){
    if (bin_read == 0){
        if(msg->bins.size() != 0){
            for(int i = 0; i<int(msg->bins.size()); i++){
                for( int j = 0; j< int(msg->bins[i].parts.size()); j++){
                    CompetitorControlSystem::bin_set_quantity_for_this_part(msg->bins[i].bin_number, msg->bins[i].parts[j].part.type, msg->bins[i].parts[j].part.color, msg->bins[i].parts[j].quantity);
                    RCLCPP_INFO(this->get_logger(),"Added Bin Part Quantity: %d", CompetitorControlSystem::bin_get_quantity_for_this_part(msg->bins[i].bin_number, msg->bins[i].parts[j].part.type, msg->bins[i].parts[j].part.color));
                    RCLCPP_INFO_STREAM(get_logger(),"Added Bin Part type:"<< PartTypetoString( msg->bins[i].parts[j].part.type));
                    RCLCPP_INFO_STREAM(get_logger(),"Added Bin Part Colour:"<< PartColortoString( msg->bins[i].parts[j].part.color));
                }
            }
        }
        bin_read++;
    }
}

void CompetitorControlSystem::conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg){
    // This if statement is used to read the the conveyor topic only once. To be removed iN RWA3. 
    if(conveyor_read == 0){
        if (msg->parts.size() != 0){
            // add variable to update values here. 
            RCLCPP_INFO_STREAM(get_logger(),"Reading Conveyor Part "<< PartTypetoString(msg->parts[0].part.type));
            RCLCPP_INFO_STREAM(get_logger(),"Reading Conveyor Part "<< PartTypetoString(msg->parts[1].part.type));
            for( int i = 0; i<int(msg->parts.size()); i++){
                CompetitorControlSystem::conveyor_set_quantity_for_this_part(msg->parts[i].part.type, msg->parts[i].part.color, msg->parts[i].quantity);
                total_parts_for_conveyor = total_parts_for_conveyor + msg->parts[i].quantity; 
                RCLCPP_INFO(this->get_logger(),"Added Conveyor Part Quantity: %d", msg->parts[i].quantity);
                RCLCPP_INFO_STREAM(get_logger(),"Added Conveyor Part type:"<< PartTypetoString(msg->parts[i].part.type));
                RCLCPP_INFO_STREAM(get_logger(),"Added Conveyor Part Colour:"<< PartColortoString( msg->parts[i].part.color));
            }
        }
        conveyor_read++;
    }
    // RCLCPP_INFO(this->get_logger(),"Total Conveyor Parts : %d", total_parts_for_conveyor);
}

bool CompetitorControlSystem::StartCompetition(){
    
    while (competition_state_ != ariac_msgs::msg::CompetitionState::READY) {}

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    client = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    RCLCPP_INFO(this->get_logger(),"Starting Competition! ");
    auto result = client->async_send_request(request);
    result.wait();

    return result.get()->success;
}

bool CompetitorControlSystem::SubmitOrder(std::string order_id)
{
    rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
    client = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
    auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
    request->order_id = order_id;
    auto result = client->async_send_request(request);
    RCLCPP_INFO_STREAM(get_logger()," Submitting Order  : " << order_id);
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    return result.get()->success;
}

void CompetitorControlSystem::EndCompetition(){
    // Service client to End competition.
    RCLCPP_INFO(this->get_logger(),"Ending Competition");
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    client = this->create_client<std_srvs::srv::Trigger>("/ariac/end_competition");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result =client->async_send_request(request);
    if(result.get()->success)
        RCLCPP_INFO(this->get_logger(),"Competition Ended");
}

bool CompetitorControlSystem::InsufficientPartsChallange(OrderData current_order_){
    
    if (current_order_.type == ariac_msgs::msg::Order::KITTING){
        
        kitting_part_details[1] = std::make_pair(std::make_pair(0,0),0);
        kitting_part_details[2] = std::make_pair(std::make_pair(0,0),0);
        kitting_part_details[3] = std::make_pair(std::make_pair(0,0),0);
        kitting_part_details[4] = std::make_pair(std::make_pair(0,0),0);

        RCLCPP_INFO_STREAM(get_logger(),"Checking insufficient parts challange for Kitting task.");
        int insuf_part_kitting=4;
        for (auto j = 0; j < current_order_.kitting.kitting_parts.number_of_parts; j ++){
            for (int i =0; i<8; i++){
                if (bin_dictionary[i][current_order_.kitting.kitting_parts.parts_[j].type][current_order_.kitting.kitting_parts.parts_[j].color] > 0 && insuf_part_kitting!= 0){
                    kitting_part_details[current_order_.kitting.kitting_parts.parts_[j].quadrant].first = std::make_pair(current_order_.kitting.kitting_parts.parts_[j].type,current_order_.kitting.kitting_parts.parts_[j].color);
                    kitting_part_details[current_order_.kitting.kitting_parts.parts_[j].quadrant].second = 1;
                    bin_set_quantity_for_this_part(i, current_order_.kitting.kitting_parts.parts_[j].type, current_order_.kitting.kitting_parts.parts_[j].color, -1);
                    insuf_part_kitting --;
                }
            }
            if (insuf_part_kitting != 0){
                if (conveyor_dictionary[current_order_.kitting.kitting_parts.parts_[j].type][current_order_.kitting.kitting_parts.parts_[j].color] >=1){
                    kitting_part_details[current_order_.kitting.kitting_parts.parts_[j].quadrant].first = std::make_pair(current_order_.kitting.kitting_parts.parts_[j].type,current_order_.kitting.kitting_parts.parts_[j].color);
                    kitting_part_details[current_order_.kitting.kitting_parts.parts_[j].quadrant].second = 1;
                    conveyor_set_quantity_for_this_part(current_order_.kitting.kitting_parts.parts_[j].type, current_order_.kitting.kitting_parts.parts_[j].color, -1);
                    insuf_part_kitting --;
                }         
            }
        }
        if (insuf_part_kitting != 0){
            RCLCPP_INFO_STREAM(get_logger()," Insufficient Parts to complete Kitting Order. Incomplete order will be submitted.");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        }
        else{
            RCLCPP_INFO_STREAM(get_logger()," Sufficient Parts available to complete Kitting Order.");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        }
    }

    else if(current_order_.type == ariac_msgs::msg::Order::ASSEMBLY){
        RCLCPP_INFO_STREAM(get_logger(),"Checking insufficient parts challange for Assembly task.");
        int insuf_part_assembly = 4;
        // for assembly task
        if(current_order_.assembly.agv_numbers.size() == 2 && insuf_part_assembly != 0){
            
                assembly_part_details[current_order_.assembly.agv_numbers[0]][1]= std::make_pair(current_order_.assembly.assembly_parts.parts_[0].type,current_order_.assembly.assembly_parts.parts_[0].color);
                assembly_part_details[current_order_.assembly.agv_numbers[0]][2]= std::make_pair(current_order_.assembly.assembly_parts.parts_[1].type,current_order_.assembly.assembly_parts.parts_[1].color);
                
                assembly_part_details[current_order_.assembly.agv_numbers[1]][1]= std::make_pair(current_order_.assembly.assembly_parts.parts_[2].type,current_order_.assembly.assembly_parts.parts_[2].color);
                assembly_part_details[current_order_.assembly.agv_numbers[1]][2]= std::make_pair(current_order_.assembly.assembly_parts.parts_[3].type,current_order_.assembly.assembly_parts.parts_[3].color);

                insuf_part_assembly = 0;
            }

        else if(current_order_.assembly.agv_numbers.size() == 1  && insuf_part_assembly != 0){             
                assembly_part_details[current_order_.assembly.agv_numbers[0]][1]= std::make_pair(current_order_.assembly.assembly_parts.parts_[0].type,current_order_.assembly.assembly_parts.parts_[0].color);
                assembly_part_details[current_order_.assembly.agv_numbers[0]][2]= std::make_pair(current_order_.assembly.assembly_parts.parts_[1].type,current_order_.assembly.assembly_parts.parts_[1].color);
                assembly_part_details[current_order_.assembly.agv_numbers[0]][3]= std::make_pair(current_order_.assembly.assembly_parts.parts_[2].type,current_order_.assembly.assembly_parts.parts_[2].color);
                assembly_part_details[current_order_.assembly.agv_numbers[0]][4]= std::make_pair(current_order_.assembly.assembly_parts.parts_[3].type,current_order_.assembly.assembly_parts.parts_[3].color);  
                insuf_part_assembly = 0;
        }
        if (insuf_part_assembly != 0){
            RCLCPP_INFO_STREAM(get_logger()," Insufficient Parts to complete Assembly Order. Incomplete order will be submitted.");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        } else{
            RCLCPP_INFO_STREAM(get_logger()," Sufficient Parts available to complete Assembly Order.");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    }   }

    else if(current_order_.type == ariac_msgs::msg::Order::COMBINED){
        RCLCPP_INFO_STREAM(get_logger(),"Checking insufficient parts challange for Combined task.");
        int insuf_part_kitting=4;
        // RCLCPP_INFO(this->get_logger(), "Quantity of parts = %d", current_order_.kitting.kitting_parts.number_of_parts);
        for (auto j = 0; j < current_order_.combined.combined_parts.number_of_parts; j ++){
            for (int i =0; i<8; i++){
                if (bin_dictionary[i][current_order_.combined.combined_parts.parts_[j].type][current_order_.combined.combined_parts.parts_[j].color] > 0 && insuf_part_kitting!= 0){
                    kitting_part_details[insuf_part_kitting].first = std::make_pair(current_order_.combined.combined_parts.parts_[j].type,current_order_.combined.combined_parts.parts_[j].color);
                    kitting_part_details[insuf_part_kitting].second = 1;
                    bin_set_quantity_for_this_part(i, current_order_.combined.combined_parts.parts_[j].type, current_order_.combined.combined_parts.parts_[j].color, -1);
                    insuf_part_kitting --;
                    }
            }
            if (insuf_part_kitting != 0){
                if (conveyor_dictionary[current_order_.combined.combined_parts.parts_[j].type][current_order_.combined.combined_parts.parts_[j].color] >=1){
                    kitting_part_details[insuf_part_kitting].first = std::make_pair(current_order_.combined.combined_parts.parts_[j].type,current_order_.combined.combined_parts.parts_[j].color);
                    kitting_part_details[insuf_part_kitting].second = 1;
                    conveyor_set_quantity_for_this_part(current_order_.combined.combined_parts.parts_[j].type, current_order_.combined.combined_parts.parts_[j].color, -1);
                    insuf_part_kitting --;
                }
            }
        }
        if (insuf_part_kitting != 0)
            RCLCPP_INFO_STREAM(get_logger()," Insufficient Parts to complete Combined Order. Incomplete order will be submitted.");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    }
    
    return true;
}

void CompetitorControlSystem::CombinedTaskAssemblyUpdate(CombinedInfo task){   
    assembly_part_details[task.station][1]= std::make_pair(task.combined_parts.parts_[0].type,task.combined_parts.parts_[0].color);
    assembly_part_details[task.station][2]= std::make_pair(task.combined_parts.parts_[1].type,task.combined_parts.parts_[1].color);
    assembly_part_details[task.station][3]= std::make_pair(task.combined_parts.parts_[2].type,task.combined_parts.parts_[2].color);
    assembly_part_details[task.station][4]= std::make_pair(task.combined_parts.parts_[3].type,task.combined_parts.parts_[3].color);  
}

void CompetitorControlSystem::floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  floor_gripper_state_ = *msg;
}

void CompetitorControlSystem::ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  ceiling_gripper_state_ = *msg;
}

void CompetitorControlSystem::FloorRobotSendHome(){
    RCLCPP_INFO_STREAM(get_logger()," Moving Floor Robot to Home position");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    // Move floor robot to home joint state
    floor_robot_.setNamedTarget("home");
    FloorRobotMovetoTarget();
}

bool CompetitorControlSystem::LockAGVTray(int agv_num)
{   
    RCLCPP_INFO_STREAM(get_logger()," Locking AGV for Assembly Task");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

    std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

    client = this->create_client<std_srvs::srv::Trigger>(srv_name);

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        
    auto result =client->async_send_request(request);
    result.wait();
    
    return result.get()->success;
}

bool CompetitorControlSystem::FloorRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(floor_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }

}

geometry_msgs::msg::Pose CompetitorControlSystem::MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
{
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1*f2;
    
    return tf2::toMsg(f3);
}

double CompetitorControlSystem::GetYaw(geometry_msgs::msg::Pose pose)
{
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

geometry_msgs::msg::Pose CompetitorControlSystem::BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;

  return pose;
}

geometry_msgs::msg::Pose CompetitorControlSystem::FrameWorldPose(std::string frame_id){
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Could not get transform");
  }

  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  return pose;
}

bool CompetitorControlSystem::FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9) {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }
        
    // Retime trajectory 
    robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
    rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(floor_robot_.execute(trajectory));
}

geometry_msgs::msg::Quaternion CompetitorControlSystem::SetRobotOrientation(double rotation)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);
    
    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q; 
}

void CompetitorControlSystem::FloorRobotWaitForAttach(double timeout){
    // Wait for part to be attached
    rclcpp::Time start = now();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;

    while (!floor_gripper_state_.attached) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout)){
        RCLCPP_ERROR(get_logger(), "Unable to pick up object");
        return;
        }
    }
}

void CompetitorControlSystem::FloorRobotWaitForAttachPump(double timeout){
    std::vector<geometry_msgs::msg::Pose> waypoints;
    bool movement{true};
    
    while(movement && !floor_gripper_state_.attached){
        geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;
        waypoints.clear();
        starting_pose.position.z -= 0.0001;
        waypoints.push_back(starting_pose);
        movement = FloorRobotMoveCartesian(waypoints, 0.1, 0.1);
        RCLCPP_ERROR(get_logger(), "Inside while loop pickup");
    }

    rclcpp::Time start = now();
    geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose;
    while (!floor_gripper_state_.attached) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

        waypoints.clear();
        starting_pose.position.z -= 0.00025;
        waypoints.push_back(starting_pose);

        FloorRobotMoveCartesian(waypoints, 0.1, 0.1);

        usleep(200);

        if (now() - start > rclcpp::Duration::from_seconds(timeout)){
        RCLCPP_ERROR(get_logger(), "Unable to pick up object");
    }
    }
}

bool CompetitorControlSystem::FloorRobotSetGripperState(bool enable)
{
    if (floor_gripper_state_.enabled == enable) {
        if (floor_gripper_state_.enabled)
            RCLCPP_INFO(get_logger(), "Already enabled");
        else 
            RCLCPP_INFO(get_logger(), "Already disabled");
        
        return false;
    }
    // Call enable service
    auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
    request->enable = enable;

    auto result = floor_robot_gripper_enable_->async_send_request(request);
    result.wait();

    if (!result.get()->success) {
        RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
        return false;
    }

    return true;
}

void CompetitorControlSystem::AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
    moveit_msgs::msg::CollisionObject collision;

    collision.id = name;
    collision.header.frame_id = "world";

    shape_msgs::msg::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("test_competitor");
    std::stringstream path;
    path << "file://" << package_share_directory << "/meshes/" << mesh_file;
    std::string model_path = path.str();

    shapes::Mesh *m = shapes::createMeshFromResource(model_path);
    shapes::constructMsgFromShape(m, mesh_msg);

    mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

    collision.meshes.push_back(mesh);
    collision.mesh_poses.push_back(model_pose);

    collision.operation = collision.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision);

    planning_scene_.addCollisionObjects(collision_objects);
}

bool CompetitorControlSystem::FloorRobotPickandPlaceTray(int tray_id, int agv_num){
    // Checking for parts in bins/tables using camera
    RCLCPP_INFO (this->get_logger()," Checking Tray Tables for Tray Id: '%d' ", tray_id);
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");

    // Check if kit tray is on one of the two tables
    geometry_msgs::msg::Pose tray_pose;
    std::string station;
    bool found_tray = false;

    // Check table 1
    for (auto tray: kts1_trays_) {
        if (tray.id == tray_id) {
        station = "kts1";
        tray_pose = MultiplyPose(kts1_camera_pose_, tray.pose);
        found_tray = true;
        break;
        }
    }
    // Check table 2
    if (!found_tray) {
        for (auto tray: kts2_trays_) {
        if (tray.id == tray_id) {
            station = "kts2";
            tray_pose = MultiplyPose(kts2_camera_pose_, tray.pose);
            found_tray = true;
            break;
        }
        }
    }
    if (!found_tray)
        return false;

    double tray_rotation = GetYaw(tray_pose);

    RCLCPP_INFO(this->get_logger()," Moving Robot to Tray Table");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        // Move floor robot to the corresponding kit tray table
    if (station == "kts1") {
        floor_robot_.setJointValueTarget(floor_kts1_js_);
    } else {
        floor_robot_.setJointValueTarget(floor_kts2_js_);
    }
    FloorRobotMovetoTarget();

    RCLCPP_INFO (this->get_logger()," Checking if the Floor Robot has Tray gripper");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    // Change gripper to tray gripper
    if (floor_gripper_state_.type != "tray_gripper") {
        FloorRobotChangeGripper(station, "trays");
    }
    else {
        RCLCPP_INFO(this->get_logger()," Floor Robot gripper is correct");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    }

    // Move to tray
    RCLCPP_INFO(this->get_logger()," Moving to pickup tray ID: '%d'", tray_id);   
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y, 
        tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y, 
        tray_pose.position.z + pick_offset_, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    FloorRobotSetGripperState(true);
    FloorRobotWaitForAttach(3.0);

    RCLCPP_INFO(this->get_logger()," Placing tray on AGV ID: '%d'", agv_num);
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    
    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray_id);
    AddModelToPlanningScene(tray_name, "kit_tray.stl", tray_pose);
    floor_robot_.attachObject(tray_name);

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(tray_pose.position.x, tray_pose.position.y, 
        tray_pose.position.z + 0.2, SetRobotOrientation(tray_rotation)));
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_num)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);

    FloorRobotMovetoTarget();

    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_num) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y, 
        agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));
    
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y, 
        agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));
    
    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    FloorRobotSetGripperState(false);

    floor_robot_.detachObject(tray_name);

    LockAGVTray(agv_num);

    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y, 
        agv_tray_pose.position.z + 0.3, SetRobotOrientation(0)));
    
    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    return true;
}

bool CompetitorControlSystem::FloorRobotChangeGripper(std::string station, std::string gripper_type){
    RCLCPP_INFO(this->get_logger()," Changing Floor Robot Gripper to type '%s'", gripper_type.c_str() );
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    
    // Move gripper into tool changer
    auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
        tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));
    
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
        tc_pose.position.z, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1)) 
        return false;

    // Call service to change gripper
    auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
    
    if (gripper_type == "trays") {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
    } else if (gripper_type == "parts") {
        request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
    }

    auto result =floor_robot_tool_changer_->async_send_request(request);
    result.wait();
    if (!result.get()->success) {
        RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
        return false;
    }

    waypoints.clear();
    waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
        tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

    if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1)) 
        return false;

    return true;
}

bool CompetitorControlSystem::FloorRobotPickBinPart( std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part){
    uint8_t part_type = part.first.first;
    uint8_t part_color = part.first.second;

    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;
    bool found_part = false;
    std::string bin_side;

    // Check right bins
    for (auto part: right_bins_parts_) {
        if (part.part.type == part_type && part.part.color == part_color) {
            part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
            found_part = true;
            bin_side = "right_bins";
            break;
        }
    } 

    // Check left bins
    if (!found_part) {
        for (auto part: left_bins_parts_) {
        if (part.part.type == part_type && part.part.color == part_color) {
        part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
        found_part = true;
        bin_side = "left_bins";
        break;
        }
    }
        
    }
    if (!found_part) {
        RCLCPP_ERROR(get_logger(), "Unable to locate part");
        return false;
    }

    double part_rotation = GetYaw(part_pose);

    // Change gripper at location closest to part
    if (floor_gripper_state_.type != "part_gripper") {
        std::string station;
        if (part_pose.position.y < 0) {
        station = "kts1";
        } else {
        station = "kts2";
        }

        // Move floor robot to the corresponding kit tray table
        if (station == "kts1") {
        floor_robot_.setJointValueTarget(floor_kts1_js_);
        } else {
        floor_robot_.setJointValueTarget(floor_kts2_js_);
        }
        FloorRobotMovetoTarget();

        FloorRobotChangeGripper(station, "parts");
    }

    RCLCPP_INFO_STREAM(get_logger()," Attempting to pick a '" <<PartColortoString(part_color) << "' '" << PartTypetoString(part_type)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
        part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));
    
    if(part_type == ariac_msgs::msg::Part::PUMP){
        waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
        part_pose.position.z + part_heights_[part_type]+ 0.0001, SetRobotOrientation(part_rotation)));
    }
    else{
        waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
        part_pose.position.z + part_heights_[part_type] + pick_offset_, SetRobotOrientation(part_rotation)));
    }
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);
    if(part_type == ariac_msgs::msg::Part::PUMP){
        FloorRobotWaitForAttachPump(2.0);
    }
    else
        FloorRobotWaitForAttach(2.0);

    // Add part to planning scene
    std::string part_name = part_colors_[part_color] + "_" + part_types_[part_type];
    AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    floor_robot_attached_part_.type = part_type;
    floor_robot_attached_part_.color = part_color;

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
        part_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    return true;
}

bool CompetitorControlSystem::CheckFaultyPart(std::string order_id, int quadrant){
    // Checking quality
    auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
    request->order_id = order_id;
    auto result = quality_checker_->async_send_request(request);
    result.wait();
    // Leaving comments for future testing of faulty parts detection and reference. 
    // RCLCPP_INFO_STREAM(get_logger(),"Checking Faulty part for quadrant'"<< (std::to_string(quadrant))<<"'");
    // RCLCPP_INFO_STREAM(get_logger()," Performing Quality Check: " << order_id);
    // RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    // RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    // RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    // RCLCPP_INFO_STREAM(get_logger()," Result: " << result.get()->valid_id);
    // RCLCPP_INFO_STREAM(get_logger()," Result: " << result.get()->all_passed);
    // RCLCPP_INFO_STREAM(get_logger()," Result: " << result.get()->incorrect_tray);
    // RCLCPP_INFO_STREAM(get_logger()," Result All Passed Q1: " << result.get()->quadrant1.all_passed);
    // RCLCPP_INFO_STREAM(get_logger()," Result Faulty Part Q1: " << result.get()->quadrant1.faulty_part);
    // RCLCPP_INFO_STREAM(get_logger()," Result Faulty Part Q2: " << result.get()->quadrant2.faulty_part);
    // RCLCPP_INFO_STREAM(get_logger()," Result Faulty Part Q3: " << result.get()->quadrant3.faulty_part);
    // RCLCPP_INFO_STREAM(get_logger()," Result Faulty Part Q4: " << result.get()->quadrant4.faulty_part);
    usleep(2500); 
    switch (quadrant)
    {
    case 1:
        return (result.get()->quadrant1.faulty_part);

    case 2:
        return (result.get()->quadrant2.faulty_part);

    case 3:
        return (result.get()->quadrant3.faulty_part);

    case 4:
        return (result.get()->quadrant4.faulty_part);

    default:
        RCLCPP_INFO_STREAM(get_logger(),"Unable to get faulty part result, returning false for quadrant '"<< (std::to_string(quadrant))<<"'");
        return false;

    }
}

bool CompetitorControlSystem::ThrowFaultyPartInBin(){
    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto current_pose = floor_robot_.getCurrentPose().pose;

    current_pose.position.z += 0.2;
    waypoints.push_back(current_pose);
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    floor_robot_.setJointValueTarget(floor_waste_bin);
    FloorRobotMovetoTarget();
    FloorRobotSetGripperState(false);

    return true; 
}

bool CompetitorControlSystem::FloorRobotPlacePartOnKitTray(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part, int tray_id, uint8_t agv_no, std::string order_id ){
    uint8_t part_type = part.first.first;
    uint8_t part_color = part.first.second;
    // uint8_t part_location = part.second;
    RCLCPP_INFO_STREAM(get_logger()," Move robot to the AGV number : '"<< (std::to_string(agv_no))<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    
    RCLCPP_INFO_STREAM(get_logger()," Place '"<< PartColortoString(part_color)<<"' '"<<PartTypetoString(part_type)<<"' in quadrant '"<<std::to_string(quadrant)<< "' on the kit tray ID: '"<< std::to_string(tray_id)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");

    if (!floor_gripper_state_.attached) {
        RCLCPP_ERROR(get_logger(), "No part attached");
        return false;
    }

    // Move to agv
    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_no)]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    // Determine target pose for part based on agv_tray pose
    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv_no) + "_tray");

    auto part_drop_offset = BuildPose(quad_offsets_[quadrant].first, quad_offsets_[quadrant].second, 0.0, 
        geometry_msgs::msg::Quaternion());

    auto part_drop_pose = MultiplyPose(agv_tray_pose, part_drop_offset);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
        part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
        part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type],
        SetRobotOrientation(0)));
    
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    waypoints.clear(); 

    bool faulty_part{false};
    bool movement{true}; 
    while(movement && !faulty_part && !faulty_part_discarded_flag){
        RCLCPP_INFO_STREAM(get_logger(),"Moving down to test faulty part");
        geometry_msgs::msg::Pose starting_pose = floor_robot_.getCurrentPose().pose; 
        starting_pose.position.z -= 0.001;
        waypoints.push_back(starting_pose);
        movement = FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
        waypoints.clear(); 
        // Check if the current part is faulty 
        if(movement)
            faulty_part = CheckFaultyPart(order_id, quadrant);
    }

    RCLCPP_INFO_STREAM(get_logger(),"Result of faulty part test: '"<< (std::to_string(faulty_part))<<"'");
    if (faulty_part){
        ThrowFaultyPartInBin();
        faulty_part_discarded_flag = true; 
        std::string part_name = part_colors_[floor_robot_attached_part_.color] + "_" + part_types_[floor_robot_attached_part_.type];
        floor_robot_.detachObject(part_name);
        return false;
    }

    FloorRobotSetGripperState(false);
    std::string part_name = part_colors_[floor_robot_attached_part_.color] + 
        "_" + part_types_[floor_robot_attached_part_.type];
    floor_robot_.detachObject(part_name);

    waypoints.clear();
    waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y,
        part_drop_pose.position.z + 0.3,
        SetRobotOrientation(0)));
    
    
    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

    return true;
}

int CompetitorControlSystem::ConveyorPartPickLocation(){
    int diff = breakbeam_one_counter - breakbeam_two_counter;
    if(diff >= 1)
        return 2; 
    else
        return 1; 

}

std::array<double,3> CompetitorControlSystem::BinAvailableLocation(int location){
    bool found = false; 
    if(location == 1){
        std::vector<int> not_available_indices_right;
        auto right_bins_parts_current = right_bins_parts_;
        for(auto i: right_bins_parts_current){
            for(auto j=0; j < int(right_bin.size()); j++){  
                if (std::sqrt( std::pow(right_bin.at(j)[1] -  i.pose.position.y, 2) + std::pow(right_bin.at(j)[2] -  i.pose.position.z, 2)) <= 0.08){
                    not_available_indices_right.push_back(j);
                    found = true; 
                    break;
                }
            }
        }
        int random_num_right{};
        do {
            random_num_right = rand()%18;
        } while (std::find(not_available_indices_right.begin(), not_available_indices_right.end(), random_num_right) != not_available_indices_right.end());
        
        std::array<double,3> pose{{right_bin.at(random_num_right).at(0), right_bin.at(random_num_right).at(1), right_bin.at(random_num_right).at(2)}};
        return pose ;  
    }

    else if((location == 2) || !found){
        std::vector<int> not_available_indices_left;
        auto left_bins_parts_current = left_bins_parts_;
        for(auto i: left_bins_parts_current){
            for(auto j=0; j < int(left_bin.size()); j++){  
                if (std::sqrt( std::pow(left_bin.at(j)[1] -  i.pose.position.y, 2) + std::pow(left_bin.at(j)[2] -  i.pose.position.z, 2)) <= 0.08){
                    not_available_indices_left.push_back(j);
                    found = true;
                    break;
                }
            }
        }
        int random_num_left{};
        do {
            random_num_left = rand()%18;
        } while (std::find(not_available_indices_left.begin(), not_available_indices_left.end(), random_num_left) != not_available_indices_left.end());
        
        std::array<double,3> pose{{left_bin.at(random_num_left).at(0), left_bin.at(random_num_left).at(1), left_bin.at(random_num_left).at(2)}};
        return pose ;
    }
    std::array<double,3> pose{{0, 0, 1.0}};
    return pose ;
}

bool CompetitorControlSystem::FloorRobotConveyorPartspickup(int location){
    if(location == 1){
        floor_robot_.setJointValueTarget(floor_conveyor_parts_pickup_1);
        FloorRobotMovetoTarget();
    }
    else{
        floor_robot_.setJointValueTarget(floor_conveyor_parts_pickup_2);
        FloorRobotMovetoTarget();
    }
    geometry_msgs::msg::Pose part_pose;
    geometry_msgs::msg::Pose part_pose_stl;

    uint8_t part_type;
    uint8_t part_color;
    geometry_msgs::msg::Pose conv_camera_pose_copy = conv_camera_pose_;
    
    // Deciding location where part needs to be picked up
    std::vector<geometry_msgs::msg::Pose> waypoints;

    while(!conveyor_camera_received_data){}
    while(conv_part_.size() == 0){}

    RCLCPP_INFO_STREAM(get_logger()," Selected pickup location is: '"<< (std::to_string(location))<<"'");
    
    if (location == 1){
        conv_part_current.push_back(conv_part_[breakbeam_one_counter]);
        part_pose = MultiplyPose(conv_camera_pose_copy, conv_part_current[0].pose);
        part_type = conv_part_current[0].part.type;
        part_color = conv_part_current[0].part.color;
        waypoints.push_back(BuildPose(part_pose.position.x, conveyor_pickup_location_one,
            part_pose.position.z + part_heights_[part_type] + pick_offset_-0.001, SetRobotOrientation(0)));
        part_pose_stl = BuildPose(part_pose.position.x, conveyor_pickup_location_one,
            part_pose.position.z + part_heights_[part_type] + pick_offset_-0.001, SetRobotOrientation(0)); 
        conv_part_counter_breakbeam1++; 
    }
    else if (location == 2){
        // conv_part_current.push_back(conv_part_[breakbeam_one_counter - breakbeam_two_counter -conv_part_counter_breakbeam1]);
        conv_part_current.push_back(conv_part_[breakbeam_two_counter + conv_part_counter_breakbeam1]);
        part_pose = MultiplyPose(conv_camera_pose_copy, conv_part_current[0].pose);
        part_type = conv_part_current[0].part.type;
        part_color = conv_part_current[0].part.color;
        waypoints.push_back(BuildPose(part_pose.position.x, conveyor_pickup_location_two,
            part_pose.position.z + part_heights_[part_type] + pick_offset_-0.001, SetRobotOrientation(0)));
        part_pose_stl = BuildPose(part_pose.position.x, conveyor_pickup_location_two,
            part_pose.position.z + part_heights_[part_type] + pick_offset_-0.001, SetRobotOrientation(0)); 
    }
    
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    
    FloorRobotSetGripperState(true);
    waypoints.clear();

    if ((breakbeam_two_counter + conv_part_counter_breakbeam1) == total_parts_for_conveyor){
        return false; 
    }
    std::string part_name = part_colors_[part_color] + "_" + part_types_[part_type];
    if (location == 1){
        breakbeam_start_received_data = false; 
        while (breakbeam_start_received_data == false){}
        FloorRobotWaitForAttach(2.0); 

        // Add part to planning scene
        AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose_stl);
        floor_robot_.attachObject(part_name);
        floor_robot_attached_part_ = conv_part_current[0].part;
        breakbeam_start_received_data = false; 
        waypoints.push_back(BuildPose(part_pose.position.x, conveyor_pickup_location_one, 
            part_pose.position.z + 0.5, SetRobotOrientation(0)));
    }
        
    else if (location == 2){
        breakbeam_end_received_data = false;
        while (breakbeam_end_received_data == false){}
        
        FloorRobotWaitForAttach(2.0); 
        // Add part to planning scene
        AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose_stl);
        floor_robot_.attachObject(part_name);
        floor_robot_attached_part_ = conv_part_current[0].part;
        breakbeam_end_received_data = false;
        waypoints.push_back(BuildPose(part_pose.position.x, conveyor_pickup_location_two, 
            part_pose.position.z + 0.5, SetRobotOrientation(0)));
    }
    
    if (!floor_gripper_state_.attached) {
        RCLCPP_ERROR(get_logger(), "No part attached");
        return false;
    }

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    if (location == 1){
        floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["right_bins"]);
        floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
        FloorRobotMovetoTarget();
        
        waypoints.clear();

        std::array<double, 3> pose = BinAvailableLocation(location);
        geometry_msgs::msg::Pose part_drop_offset = BuildPose(pose.at(0), pose.at(1), pose.at(2), geometry_msgs::msg::Quaternion());
        geometry_msgs::msg::Pose part_drop_pose = MultiplyPose(right_bins_camera_pose_, part_drop_offset);
        waypoints.push_back(BuildPose(part_drop_pose.position.x , part_drop_pose.position.y, part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));
        waypoints.push_back(BuildPose(part_drop_pose.position.x , part_drop_pose.position.y, part_drop_pose.position.z + 0.15, SetRobotOrientation(0)));
        FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

        FloorRobotSetGripperState(false); 
        RCLCPP_INFO_STREAM(get_logger()," Placed part at pose: '"<< std::to_string(part_drop_pose.position.x)<<std::to_string( part_drop_pose.position.y ));
        floor_robot_.detachObject(part_name);
        waypoints.clear();
        waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y , part_drop_pose.position.z + 0.5, SetRobotOrientation(0)));
        FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    }
        
    else if (location == 2){
        floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_["left_bins"]);
        floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
        FloorRobotMovetoTarget();
        
        waypoints.clear();
 
        std::array<double, 3> pose = BinAvailableLocation(location);

        geometry_msgs::msg::Pose part_drop_offset = BuildPose(pose.at(0), pose.at(1), pose.at(2), geometry_msgs::msg::Quaternion());
        geometry_msgs::msg::Pose part_drop_pose = MultiplyPose(left_bins_camera_pose_, part_drop_offset);
        waypoints.push_back(BuildPose(part_drop_pose.position.x , part_drop_pose.position.y, part_drop_pose.position.z + 0.3, SetRobotOrientation(0)));
        waypoints.push_back(BuildPose(part_drop_pose.position.x , part_drop_pose.position.y, part_drop_pose.position.z + 0.15, SetRobotOrientation(0)));
        FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

        FloorRobotSetGripperState(false); 
        RCLCPP_INFO_STREAM(get_logger()," Placed part at pose: '"<< std::to_string(part_drop_pose.position.x)<<std::to_string( part_drop_pose.position.y ));
        floor_robot_.detachObject(part_name);
        waypoints.clear();
        waypoints.push_back(BuildPose(part_drop_pose.position.x, part_drop_pose.position.y , part_drop_pose.position.z + 0.5, SetRobotOrientation(0)));
        FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    }
    conv_part_current.clear();

    return true;
}

bool CompetitorControlSystem::MoveAGV(uint8_t agv, uint8_t destination){

    rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

    std::string srv_name = "/ariac/move_agv" + std::to_string(agv);

    client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
    request->location = destination;

    auto result =client->async_send_request(request);
    result.wait();

    return result.get()->success;
}

bool CompetitorControlSystem::CeilingRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(ceiling_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(ceiling_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}

bool CompetitorControlSystem::CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9) {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }
    
  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
  rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(ceiling_robot_.execute(trajectory));
}

bool CompetitorControlSystem::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  bool assembled = false;
  while (!assembled) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

    // Check if part is assembled
    switch (part.part.type) {
      case ariac_msgs::msg::Part::BATTERY:
        assembled = assembly_station_states_[station].battery_attached;
        break;
      case ariac_msgs::msg::Part::PUMP:
        assembled = assembly_station_states_[station].pump_attached;
        break;
      case ariac_msgs::msg::Part::SENSOR:
        assembled = assembly_station_states_[station].sensor_attached;
        break;
      case ariac_msgs::msg::Part::REGULATOR:
        assembled = assembly_station_states_[station].regulator_attached;
        break;
      default:
        RCLCPP_WARN(get_logger(), "Not a valid part type");
        return false;
    }

    double step = 0.0005;

    waypoints.clear();
    starting_pose.position.x += step * part.install_direction.x;
    starting_pose.position.y += step * part.install_direction.y;
    starting_pose.position.z += step * part.install_direction.z;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(500);

    if (now() - start > rclcpp::Duration::from_seconds(5)){
      RCLCPP_ERROR(get_logger(), "Unable to assemble object");
      ceiling_robot_.stop();
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "Part is assembled");
  
  return true;
}

void CompetitorControlSystem::CeilingRobotSendHome(){
    RCLCPP_INFO_STREAM(get_logger()," Moving Ceiling Robot to Home position");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    ceiling_robot_.setNamedTarget("home");
    CeilingRobotMovetoTarget();
}

bool CompetitorControlSystem::CeilingRobotMoveToAssemblyStation(int station)
{
    switch (station) {
        case 1:
        ceiling_robot_.setJointValueTarget(ceiling_as1_js_);
        break;
        case 2:
        ceiling_robot_.setJointValueTarget(ceiling_as2_js_);
        break;
        case 3:
        ceiling_robot_.setJointValueTarget(ceiling_as3_js_);
        break;
        case 4:
        ceiling_robot_.setJointValueTarget(ceiling_as4_js_);
        break;
        default:
        RCLCPP_WARN(get_logger(), "Not a valid assembly station");
        return false;
    }

    return CeilingRobotMovetoTarget();
}

bool CompetitorControlSystem::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
  double part_rotation = GetYaw(part.pose);
  std::vector<geometry_msgs::msg::Pose> waypoints;

  double dx = 0;
  double dy = 0;

  if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
    dx = battery_grip_offset_*cos(part_rotation);
    dy = battery_grip_offset_*sin(part_rotation);
  }

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy, 
    part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));
  
  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy, 
    part.pose.position.z + part_heights_[part.part.type] + pick_offset_, SetRobotOrientation(part_rotation)));
  
  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  CeilingRobotSetGripperState(true);

  CeilingRobotWaitForAttach(3.0);

  // Add part to planning scene
  std::string part_name = part_colors_[part.part.color] + "_" + part_types_[part.part.type];
  AddModelToPlanningScene(part_name, part_types_[part.part.type] + ".stl", part.pose);
  ceiling_robot_.attachObject(part_name);
  ceiling_robot_attached_part_ = part.part;

  // Move up slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;
  current_pose.position.z += 0.2;
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  return true;

}

bool CompetitorControlSystem::CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part)
{
    // Check that part is attached and matches part to assemble
    if (!ceiling_gripper_state_.attached) {
        RCLCPP_WARN(get_logger(), "No part attached");
        return false;
    }
        
    if (part.part != ceiling_robot_attached_part_){
        RCLCPP_WARN(get_logger(), "Incorrect part attached for this assembly");
        return false;
    }
    
    // Calculate assembled pose in world frame
    std::string insert_frame_name;
    switch (station) {
        case 1:
        insert_frame_name = "as1_insert_frame";
        break;
        case 2:
        insert_frame_name = "as2_insert_frame";
        break;
        case 3:
        insert_frame_name = "as3_insert_frame";
        break;
        case 4:
        insert_frame_name = "as4_insert_frame";
        break;
        default:
        RCLCPP_WARN(get_logger(), "Not a valid assembly station");
        return false;
    }

    // Calculate robot positions at assembly and approach
    KDL::Vector install(part.install_direction.x, part.install_direction.y, part.install_direction.z);

    KDL::Frame insert;
    tf2::fromMsg(FrameWorldPose(insert_frame_name), insert);

    KDL::Frame part_assemble;
    tf2::fromMsg(part.assembled_pose.pose, part_assemble);

    KDL::Frame part_to_gripper;

    // Build approach waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
        tf2::fromMsg(BuildPose(battery_grip_offset_, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

        KDL::Vector up(0, 0, 0.1);
        waypoints.push_back(tf2::toMsg(insert * KDL::Frame(up) * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
        waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));    

    } else {
        tf2::fromMsg(BuildPose(0, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

        waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
    }
    
    // Move to approach position
    CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

    // Move to just before assembly position
    waypoints.clear();
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.003) * part_assemble * part_to_gripper));
    CeilingRobotMoveCartesian(waypoints, 0.1, 0.1, true);

    CeilingRobotWaitForAssemble(station, part);

    CeilingRobotSetGripperState(false);

    std::string part_name = part_colors_[ceiling_robot_attached_part_.color] + 
        "_" + part_types_[ceiling_robot_attached_part_.type];
    ceiling_robot_.detachObject(part_name);

    // Move away slightly
    auto current_pose = ceiling_robot_.getCurrentPose().pose;

    if (part.part.type == ariac_msgs::msg::Part::REGULATOR) {
        current_pose.position.x -= 0.05;
    }
    else {
        current_pose.position.z += 0.1;
    }
    
    waypoints.clear();
    waypoints.push_back(current_pose);

    CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);
    
    return true;

}

bool CompetitorControlSystem::CeilingRobotSetGripperState(bool enable)
{
  if (ceiling_gripper_state_.enabled == enable) {
    if (ceiling_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else 
      RCLCPP_INFO(get_logger(), "Already disabled");
    
    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = ceiling_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success) {
    RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}

void CompetitorControlSystem::CeilingRobotWaitForAttach(double timeout)
{
 // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  while (!ceiling_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  } 
}

// int CompetitorControlSystem::AGVAvailable(int station){
//     if (station == ariac_msgs::msg::CombinedTask::AS1 || station == ariac_msgs::msg::CombinedTask::AS2){
//         if(agv2_location == ariac_msgs::msg::AGVStatus::KITTING)
//             return 2;
//         else if(agv1_location == ariac_msgs::msg::AGVStatus::KITTING)
//             return 1;
//     }
//     else if (station == ariac_msgs::msg::CombinedTask::AS3 || station == ariac_msgs::msg::CombinedTask::AS4){
//         if(agv3_location == ariac_msgs::msg::AGVStatus::KITTING)
//             return 3;
//         else if(agv4_location == ariac_msgs::msg::AGVStatus::KITTING)
//             return 4;
//     }
//     return 0;
// }

// int CompetitorControlSystem::TrayAvailable(int station){
//     // Check table 1
//     if(kts1_trays_.size()>0){
//         return kts1_trays_[0].id;
//     }
//     // Check table 2
//     else if(kts2_trays_.size()>0){
//         return kts2_trays_[0].id;
//     }
//     else 
//         return 0; 
// }

bool CompetitorControlSystem::CompleteKittingTask(OrderData current_order_)
{   
    KittingInfo task = current_order_.kitting;
    FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);
    if (priority_orders_.size() != 0 && !current_order_.importance_flag){
                    current_order_.kitting_part_details_orderdata = kitting_part_details;                   
                    current_order_.abandoned_ = true;
                    incomplete_orders_.push_back(current_order_);
                    return current_order_.abandoned_;
    }

    if(current_order_.abandoned_){
        for (auto kit_part = current_order_.kitting_part_details_orderdata.begin(); kit_part != current_order_.kitting_part_details_orderdata.end(); kit_part++){
            if(kit_part->second.second == 1 ){
                    RCLCPP_INFO_STREAM(get_logger()," Using incomplete order_data'"<< std::to_string(kit_part->second.second)<<"'");
                FloorRobotPickBinPart(kit_part->second);
                FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second,task.tray_id, task.agv_number, current_order_.id);
                if (faulty_part_discarded_flag == true){
                    FloorRobotPickBinPart(kit_part->second);
                    FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second,task.tray_id, task.agv_number, current_order_.id);
                    faulty_part_discarded_flag = false; 
                }
                kit_part->second.second = 2;

                if (priority_orders_.size() != 0 && !current_order_.importance_flag){
                    current_order_.abandoned_ = true;
                    incomplete_orders_.push_back(current_order_);
                    return current_order_.abandoned_;
                }
                else{
                    continue;
                }
            }
        }
        current_order_.abandoned_ = false; 
    }
    else{
        for (auto kit_part = kitting_part_details.begin(); kit_part != kitting_part_details.end(); kit_part++){
            if(kit_part->second.second != 0 || kit_part->second.second != 2){
                FloorRobotPickBinPart(kit_part->second);
                FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second,task.tray_id, task.agv_number, current_order_.id);
                if (faulty_part_discarded_flag == true){
                    FloorRobotPickBinPart(kit_part->second);
                    FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second,task.tray_id, task.agv_number, current_order_.id);
                    faulty_part_discarded_flag = false; 
                }
                kit_part->second.second = 2;

                if (priority_orders_.size() != 0 && !current_order_.importance_flag){
                    current_order_.kitting_part_details_orderdata = kitting_part_details;
                    current_order_.abandoned_ = true;
                    incomplete_orders_.push_back(current_order_);
                    return current_order_.abandoned_;
                }
                else{
                    continue;
                }

            }
        }
    }
    MoveAGV(task.agv_number, task.destination);
    return current_order_.abandoned_;
}

bool CompetitorControlSystem::CompleteAssemblyTask(OrderData current_order_)
{   
    RCLCPP_INFO_STREAM(get_logger()," Executing Assembly Task ");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    AssemblyInfo task = current_order_.assembly;
    
    if(!current_order_.agv_part_poses_extracted){
        for ( int j = 0; j < int(task.agv_numbers.size()); j++){
            LockAGVTray(task.agv_numbers[j]);
            int destination;
            if ((task.station == ariac_msgs::msg::AssemblyTask::AS1) || (task.station == ariac_msgs::msg::AssemblyTask::AS3)) {
                destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
            } else {
                destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
            }

            MoveAGV(task.agv_numbers[j], destination);
        }

        if (priority_orders_.size() != 0 && !current_order_.importance_flag){ 
            current_order_.abandoned_ = true;
            incomplete_orders_.push_back(current_order_);
            return current_order_.abandoned_;
        }

        CeilingRobotMoveToAssemblyStation(task.station);

        if (priority_orders_.size() != 0 && !current_order_.importance_flag){
            current_order_.abandoned_ = true;
            incomplete_orders_.push_back(current_order_);
            return current_order_.abandoned_;
        }
    }

    // Get Assembly Poses
    auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
    request->order_id = current_order_.id;
    auto result = pre_assembly_poses_getter_->async_send_request(request);
    
    result.wait();

    std::vector<ariac_msgs::msg::PartPose> agv_part_poses;

    if(!current_order_.agv_part_poses_extracted){
        if (result.get()->valid_id) {
            agv_part_poses = result.get()->parts;
            current_order_.agv_part_poses_extracted = true;
            if (agv_part_poses.size() == 0) {
            RCLCPP_WARN(get_logger(), "No part poses recieved");
            return false;
            }
        } else {
            RCLCPP_WARN(get_logger(), "Not a valid order ID");
            return false;
        }
    }
    else{
        agv_part_poses = assembly_agv_part_poses.front();
        assembly_agv_part_poses.erase(assembly_agv_part_poses.begin());
    }

    if(current_order_.abandoned_){
        int i = 0; 
        for (auto const &part_to_assemble : task.assembly_parts.parts_) {
            if(current_order_.status_of_assembly.at(i)==1){
                // Check if matching part exists in agv_parts
                bool part_exists = false;
                ariac_msgs::msg::PartPose part_to_pick;
                part_to_pick.part.color = part_to_assemble.color;
                part_to_pick.part.type = part_to_assemble.type;
                for (auto const &agv_part: agv_part_poses) {
                    if (agv_part.part.type == part_to_assemble.type && agv_part.part.color == part_to_assemble.color) {
                        part_exists = true;
                        part_to_pick.pose = agv_part.pose;
                        break;
                    }
                }
                if (!part_exists) {
                RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.type << 
                    " and color: " << part_to_assemble.color << " not found on tray");
                continue;
                }

                // Pick up part
                CeilingRobotPickAGVPart(part_to_pick);

                CeilingRobotMoveToAssemblyStation(task.station);
                ariac_msgs::msg::AssemblyPart part;
                
                part.part.type = part_to_assemble.type;
                part.part.color = part_to_assemble.color;
                part.assembled_pose.header.stamp.sec = part_to_assemble.header_stamp_sec;
                part.assembled_pose.header.stamp.nanosec = part_to_assemble.header_stamp_nanos;
                part.assembled_pose.header.frame_id = part_to_assemble.frame_id;
                part.assembled_pose.pose.position.x = part_to_assemble.position_x;
                part.assembled_pose.pose.position.y = part_to_assemble.position_y;
                part.assembled_pose.pose.position.z = part_to_assemble.position_z;
                part.assembled_pose.pose.orientation.x = part_to_assemble.orientation_x;
                part.assembled_pose.pose.orientation.y = part_to_assemble.orientation_y;
                part.assembled_pose.pose.orientation.z = part_to_assemble.orientation_z;
                part.assembled_pose.pose.orientation.w = part_to_assemble.orientation_w;
                part.install_direction.x = part_to_assemble.install_direction_x;
                part.install_direction.y = part_to_assemble.install_direction_y;
                part.install_direction.z = part_to_assemble.install_direction_z;
                // Assemble Part to insert
                CeilingRobotAssemblePart(task.station, part);

                CeilingRobotMoveToAssemblyStation(task.station);

                current_order_.status_of_assembly.at(i) = 2;

                if (priority_orders_.size() != 0 && !current_order_.importance_flag){                  
                assembly_agv_part_poses.push_back(agv_part_poses); 
                current_order_.abandoned_ = true;
                incomplete_orders_.push_back(current_order_);
                return current_order_.abandoned_;
                }
                i++; 
            }
            else{
                i++;
            }
        }
        current_order_.abandoned_=false; 
    }
    else{
        int i = 0;
        for (auto const &part_to_assemble : task.assembly_parts.parts_) {
            // Check if matching part exists in agv_parts
            bool part_exists = false;
            ariac_msgs::msg::PartPose part_to_pick;
            part_to_pick.part.color = part_to_assemble.color;
            part_to_pick.part.type = part_to_assemble.type;
            for (auto const &agv_part: agv_part_poses) {
                if (agv_part.part.type == part_to_assemble.type && agv_part.part.color == part_to_assemble.color) {
                    part_exists = true;
                    part_to_pick.pose = agv_part.pose;
                    break;
                }
                if (priority_orders_.size() != 0 ){
                    
                }
            }
            if (!part_exists) {
            RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.type << 
                " and color: " << part_to_assemble.color << " not found on tray");
            continue;
            }

            // Pick up part
            CeilingRobotPickAGVPart(part_to_pick);

            CeilingRobotMoveToAssemblyStation(task.station);
            ariac_msgs::msg::AssemblyPart part;
            
            part.part.type = part_to_assemble.type;
            part.part.color = part_to_assemble.color;
            part.assembled_pose.header.stamp.sec = part_to_assemble.header_stamp_sec;
            part.assembled_pose.header.stamp.nanosec = part_to_assemble.header_stamp_nanos;
            part.assembled_pose.header.frame_id = part_to_assemble.frame_id;
            part.assembled_pose.pose.position.x = part_to_assemble.position_x;
            part.assembled_pose.pose.position.y = part_to_assemble.position_y;
            part.assembled_pose.pose.position.z = part_to_assemble.position_z;
            part.assembled_pose.pose.orientation.x = part_to_assemble.orientation_x;
            part.assembled_pose.pose.orientation.y = part_to_assemble.orientation_y;
            part.assembled_pose.pose.orientation.z = part_to_assemble.orientation_z;
            part.assembled_pose.pose.orientation.w = part_to_assemble.orientation_w;
            part.install_direction.x = part_to_assemble.install_direction_x;
            part.install_direction.y = part_to_assemble.install_direction_y;
            part.install_direction.z = part_to_assemble.install_direction_z;
            // Assemble Part to insert
            CeilingRobotAssemblePart(task.station, part);

            CeilingRobotMoveToAssemblyStation(task.station);

            current_order_.status_of_assembly.at(i) = 2;

            if (priority_orders_.size() != 0 && !current_order_.importance_flag){                  
            assembly_agv_part_poses.push_back(agv_part_poses); 
            current_order_.abandoned_ = true;
            incomplete_orders_.push_back(current_order_);
            return current_order_.abandoned_;
                }

            i++;
                    
        }
    }
    return current_order_.abandoned_;
}

bool CompetitorControlSystem::CompleteCombinedTask(OrderData current_order_){

    // int agv = AGVAvailable(task.station);
    // int tray = TrayAvailable(task.station);
    CombinedInfo task = current_order_.combined;
    int agv;
    int tray{0};
    if(!current_order_.combined_tray_placed_on_agv){
        // Decide on a tray to use. Prefer tray.id 0 for Combined task s
        bool found_tray{false}; 
        for (auto tray1: kts1_trays_) {
            if (tray1.id == tray) {
                found_tray = true;
                break;
            }
        }
        // Check table 2
        if (!found_tray) {
            for (auto tray2: kts2_trays_) {
                if (tray2.id == tray) {
                    found_tray = true;
                    break;
                }
            }
        }
        if (!found_tray){
            if (kts1_trays_.size() != 0) {
                tray = kts1_trays_[0].id;
            } else if (kts2_trays_.size() != 0) {
                tray = kts2_trays_[0].id;
            } else {
                RCLCPP_ERROR(get_logger(), "No trays available.");
                return false;
            }
        }
        current_order_.combined_tray_selected = tray; 

        // Decide which AGV to use
        if (task.station == ariac_msgs::msg::CombinedTask::AS1 or task.station == ariac_msgs::msg::CombinedTask::AS2) {
            agv = 1;
        } else {
            agv = 4;
        }
        current_order_.combined_agv_selected = agv; 

        MoveAGV(agv, ariac_msgs::srv::MoveAGV::Request::KITTING);
        if(agv == 0){
            RCLCPP_INFO_STREAM(get_logger()," No AGV available to complete order at station "<< StationtoString(task.station));
            return false; 
        }

        RCLCPP_INFO_STREAM(get_logger()," Choosing AGV : '"<< std::to_string(agv) << "' and Tray ID: '"<< std::to_string(tray)<<"' for combined task");

        FloorRobotSendHome();
        FloorRobotPickandPlaceTray(tray, agv);
        current_order_.combined_tray_placed_on_agv = true; 
    }
    else{
        agv = current_order_.combined_agv_selected; 
        tray = current_order_.combined_tray_selected; 
    }

    if (priority_orders_.size() != 0 && !current_order_.importance_flag)
    {   
        if(!current_order_.abandoned_){
            current_order_.kitting_part_details_orderdata = kitting_part_details;                   
            current_order_.abandoned_ = true;
            incomplete_orders_.push_back(current_order_);
            return current_order_.abandoned_;
        }
    }

    if(current_order_.abandoned_){
        for (auto kit_part = current_order_.kitting_part_details_orderdata.begin(); kit_part != current_order_.kitting_part_details_orderdata.end(); kit_part++){
            if(kit_part->second.second == 1 ){
                RCLCPP_INFO_STREAM(get_logger()," Using incomplete order_data'"<< std::to_string(kit_part->second.second)<<"'");
                FloorRobotPickBinPart(kit_part->second);
                FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second, tray, agv, current_order_.id);
                if (faulty_part_discarded_flag == true){
                    FloorRobotPickBinPart(kit_part->second);
                    FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second, tray, agv, current_order_.id);
                    faulty_part_discarded_flag = false; 
                }
                kit_part->second.second = 2;

                if (priority_orders_.size() != 0 && !current_order_.importance_flag){
                    current_order_.abandoned_ = true;
                    incomplete_orders_.push_back(current_order_);
                    return current_order_.abandoned_;
                }
                else{
                    continue;
                }
            }
        }
    }
    else{
        for (auto kit_part = kitting_part_details.begin(); kit_part != kitting_part_details.end(); kit_part++){
            if(kit_part->second.second != 0 || kit_part->second.second != 2){
                FloorRobotPickBinPart(kit_part->second);
                FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second, tray, agv, current_order_.id);
                if (faulty_part_discarded_flag == true){
                    FloorRobotPickBinPart(kit_part->second);
                    FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second, tray, agv, current_order_.id);
                    faulty_part_discarded_flag = false; 
                }
                kit_part->second.second = 2;
                
                if (priority_orders_.size() != 0 && !current_order_.importance_flag){
                    current_order_.kitting_part_details_orderdata = kitting_part_details;
                    current_order_.abandoned_ = true;
                    incomplete_orders_.push_back(current_order_);
                    return current_order_.abandoned_;
                }
                else{
                    continue;
                }

            }
        }
    }
    
    int destination;
    if(!current_order_.agv_part_poses_extracted){
        if (task.station == ariac_msgs::msg::CombinedTask::AS1 || task.station == ariac_msgs::msg::CombinedTask::AS3) {
            destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_FRONT;
        }  
        else {
            destination = ariac_msgs::srv::MoveAGV::Request::ASSEMBLY_BACK;
        }

        MoveAGV(agv, destination);

        if (priority_orders_.size() != 0 && !current_order_.importance_flag){ 
            current_order_.abandoned_ = true;
            incomplete_orders_.push_back(current_order_);
            return current_order_.abandoned_;
        }

        CeilingRobotMoveToAssemblyStation(task.station);

        if (priority_orders_.size() != 0 && !current_order_.importance_flag){
            current_order_.abandoned_ = true;
            incomplete_orders_.push_back(current_order_);
            return current_order_.abandoned_;
        }
    }

    // Get Assembly Poses
    auto request = std::make_shared<ariac_msgs::srv::GetPreAssemblyPoses::Request>();
    request->order_id = current_order_.id;
    auto result = pre_assembly_poses_getter_->async_send_request(request);
    
    result.wait();

    std::vector<ariac_msgs::msg::PartPose> agv_part_poses; 
    if(!current_order_.agv_part_poses_extracted){
        if (result.get()->valid_id) {
            agv_part_poses = result.get()->parts;
            current_order_.agv_part_poses_extracted = true;
            if (agv_part_poses.size() == 0) {
            RCLCPP_WARN(get_logger(), "No part poses recieved");
            return false;
            }
        } else {
            RCLCPP_WARN(get_logger(), "Not a valid order ID");
            return false;
        }
    }
    else{
        agv_part_poses = assembly_agv_part_poses.front();
        assembly_agv_part_poses.erase(assembly_agv_part_poses.begin());
    }

    if(current_order_.abandoned_){
        int i = 0; 
        for (auto const &part_to_assemble : task.combined_parts.parts_) {
            if(current_order_.status_of_assembly.at(i)==1){
                // Check if matching part exists in agv_parts
                bool part_exists = false;
                ariac_msgs::msg::PartPose part_to_pick;
                part_to_pick.part.color = part_to_assemble.color;
                part_to_pick.part.type = part_to_assemble.type;
                for (auto const &agv_part: agv_part_poses) {
                    if (agv_part.part.type == part_to_assemble.type && agv_part.part.color == part_to_assemble.color) {
                        part_exists = true;
                        part_to_pick.pose = agv_part.pose;
                        break;
                    }
                }
                if (!part_exists) {
                RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.type << 
                    " and color: " << part_to_assemble.color << " not found on tray");
                continue;
                }

                // Pick up part
                CeilingRobotPickAGVPart(part_to_pick);

                CeilingRobotMoveToAssemblyStation(task.station);
                ariac_msgs::msg::AssemblyPart part;
                
                part.part.type = part_to_assemble.type;
                part.part.color = part_to_assemble.color;
                part.assembled_pose.header.stamp.sec = part_to_assemble.header_stamp_sec;
                part.assembled_pose.header.stamp.nanosec = part_to_assemble.header_stamp_nanos;
                part.assembled_pose.header.frame_id = part_to_assemble.frame_id;
                part.assembled_pose.pose.position.x = part_to_assemble.position_x;
                part.assembled_pose.pose.position.y = part_to_assemble.position_y;
                part.assembled_pose.pose.position.z = part_to_assemble.position_z;
                part.assembled_pose.pose.orientation.x = part_to_assemble.orientation_x;
                part.assembled_pose.pose.orientation.y = part_to_assemble.orientation_y;
                part.assembled_pose.pose.orientation.z = part_to_assemble.orientation_z;
                part.assembled_pose.pose.orientation.w = part_to_assemble.orientation_w;
                part.install_direction.x = part_to_assemble.install_direction_x;
                part.install_direction.y = part_to_assemble.install_direction_y;
                part.install_direction.z = part_to_assemble.install_direction_z;
                // Assemble Part to insert
                CeilingRobotAssemblePart(task.station, part);

                CeilingRobotMoveToAssemblyStation(task.station);

                current_order_.status_of_assembly.at(i) = 2;

                if (priority_orders_.size() != 0 && !current_order_.importance_flag){
                assembly_agv_part_poses.push_back(agv_part_poses); 
                current_order_.abandoned_ = true;
                incomplete_orders_.push_back(current_order_);
                return current_order_.abandoned_;
                }
                i++; 
            }
            else{
                i++;
            }
        }
        current_order_.abandoned_=false; 
    }
    else{
        int i = 0;
        for (auto const &part_to_assemble : task.combined_parts.parts_) {
            // Check if matching part exists in agv_parts
            bool part_exists = false;
            ariac_msgs::msg::PartPose part_to_pick;
            part_to_pick.part.color = part_to_assemble.color;
            part_to_pick.part.type = part_to_assemble.type;
            for (auto const &agv_part: agv_part_poses) {
                if (agv_part.part.type == part_to_assemble.type && agv_part.part.color == part_to_assemble.color) {
                    part_exists = true;
                    part_to_pick.pose = agv_part.pose;
                    break;
                }
                if (priority_orders_.size() != 0 ){
                    
                }
            }
            if (!part_exists) {
            RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.type << 
                " and color: " << part_to_assemble.color << " not found on tray");
            continue;
            }

            // Pick up part
            CeilingRobotPickAGVPart(part_to_pick);

            CeilingRobotMoveToAssemblyStation(task.station);
            ariac_msgs::msg::AssemblyPart part;

            part.part.type = part_to_assemble.type;
            part.part.color = part_to_assemble.color;
            part.assembled_pose.header.stamp.sec = part_to_assemble.header_stamp_sec;
            part.assembled_pose.header.stamp.nanosec = part_to_assemble.header_stamp_nanos;
            part.assembled_pose.header.frame_id = part_to_assemble.frame_id;
            part.assembled_pose.pose.position.x = part_to_assemble.position_x;
            part.assembled_pose.pose.position.y = part_to_assemble.position_y;
            part.assembled_pose.pose.position.z = part_to_assemble.position_z;
            part.assembled_pose.pose.orientation.x = part_to_assemble.orientation_x;
            part.assembled_pose.pose.orientation.y = part_to_assemble.orientation_y;
            part.assembled_pose.pose.orientation.z = part_to_assemble.orientation_z;
            part.assembled_pose.pose.orientation.w = part_to_assemble.orientation_w;
            part.install_direction.x = part_to_assemble.install_direction_x;
            part.install_direction.y = part_to_assemble.install_direction_y;
            part.install_direction.z = part_to_assemble.install_direction_z;
            
            // Assemble Part to insert
            CeilingRobotAssemblePart(task.station, part);

            CeilingRobotMoveToAssemblyStation(task.station);

            current_order_.status_of_assembly.at(i) = 2;

            if (priority_orders_.size() != 0 && !current_order_.importance_flag){
            assembly_agv_part_poses.push_back(agv_part_poses); 
            current_order_.abandoned_ = true;
            incomplete_orders_.push_back(current_order_);
            return current_order_.abandoned_;
                }
            i++;
        }
    }
    return current_order_.abandoned_;

}

bool CompetitorControlSystem::CompleteOrders(){
    // Wait for first order to be published and status of bins and conveyor is read.
    while ((orders_.size() == 0) && (priority_orders_.size() == 0)) {}
    while (bin_read == 0) {}
    while (conveyor_read == 0) {}
    bool success;
    while (true) {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
        success = false;
        break;
        }

        if ((orders_.size() == 0) && (priority_orders_.size() == 0) && (incomplete_orders_.size()== 0)){
            if (competition_state_  != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
                // wait for more orders
                RCLCPP_INFO(get_logger(), " Waiting for orders...");
                while ((orders_.size() == 0) || (priority_orders_.size() == 0)) {}
            } 
            else {
                RCLCPP_INFO(get_logger(), " Completed all orders");
                CompetitorControlSystem::EndCompetition();
                success = true;
                break;
            }
        }

        if (total_parts_for_conveyor>0){
            while((breakbeam_two_counter + conv_part_counter_breakbeam1) < total_parts_for_conveyor){
                int location = ConveyorPartPickLocation(); 
                FloorRobotConveyorPartspickup(location);
            }
        }

        std::vector<OrderData> current_order_;

        if (priority_orders_.size() != 0){
            current_order_.push_back(priority_orders_.front());
            priority_orders_.erase(priority_orders_.begin());
            current_order_[0].importance_flag = true;
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(),"Starting Priority order:  '%s' ", current_order_[0].id.c_str());
        }
        else if (incomplete_orders_.size() != 0){
            current_order_.push_back(incomplete_orders_.front());
            incomplete_orders_.erase(incomplete_orders_.begin());
            current_order_[0].importance_flag = false;
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(),"Resuming Normal Order:  '%s' ", current_order_[0].id.c_str());
        }
        else{
            current_order_.push_back(orders_.front());
            orders_.erase(orders_.begin());
            current_order_[0].importance_flag = false;
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
            RCLCPP_INFO(this->get_logger()," Starting normal order: '%s' ", current_order_[0].id.c_str());
        }   

        // Insufficient Parts challange
        CompetitorControlSystem::InsufficientPartsChallange(current_order_[0]);

        if (current_order_[0].type == ariac_msgs::msg::Order::KITTING) {
            current_order_[0].abandoned_ = CompetitorControlSystem::CompleteKittingTask(current_order_[0]);
            // Submit order
            if (!current_order_[0].abandoned_){
            CompetitorControlSystem::SubmitOrder(current_order_[0].id);
            }
            current_order_.erase(current_order_.begin());
        } 
        else if (current_order_[0].type == ariac_msgs::msg::Order::ASSEMBLY) {
            current_order_[0].abandoned_ = CompetitorControlSystem::CompleteAssemblyTask(current_order_[0]);
            // Submit order
            if (!current_order_[0].abandoned_){
            CompetitorControlSystem::SubmitOrder(current_order_[0].id);
            }
            current_order_.erase(current_order_.begin());
        }
        else if (current_order_[0].type == ariac_msgs::msg::Order::COMBINED) {
            current_order_[0].abandoned_ = CompetitorControlSystem::CompleteCombinedTask(current_order_[0]);
            // Submit order
            if (!current_order_[0].abandoned_){
            CompetitorControlSystem::SubmitOrder(current_order_[0].id);
            }
            current_order_.erase(current_order_.begin());
        }
    }
    return success;
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto competitor_control_system = std::make_shared<CompetitorControlSystem>("CCS");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(competitor_control_system);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    competitor_control_system->StartCompetition();

    competitor_control_system->FloorRobotSendHome();
    // competitor_control_system->CeilingRobotSendHome();

    competitor_control_system->CompleteOrders();
    rclcpp::shutdown();
}

