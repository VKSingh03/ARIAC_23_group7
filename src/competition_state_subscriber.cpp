# include "competition_state_subscriber.hpp"
# include "order_class.hpp"
# include <std_srvs/srv/trigger.hpp>
# include <memory>
# include <vector>
# include <iostream>  

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

void CompetitorControlSystem::competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
    // Reading if competition state is READY
    competition_state_ = msg->competition_state;
    if(msg->competition_state == ariac_msgs::msg::CompetitionState::READY){
        // RCLCPP_INFO(this->get_logger(),"Reading CompetitionState READY :%d", msg->competition_state);
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
        client = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        RCLCPP_INFO(this->get_logger(),"Starting Competition! ");
        auto result =client->async_send_request(request);
    } 
}

void CompetitorControlSystem::order_callback(const ariac_msgs::msg::Order::SharedPtr msg){   
    RCLCPP_INFO_STREAM(this->get_logger(), " Received order Id: " << msg->id);
    // Creating object of order_data to store current order
    OrderData order(msg); 
    //Storing order based on priority
    if(order.priority == true){
        orders_.insert(orders_.begin(),order);
        first_priority_order+=1;
        total_orders +=1; 
        RCLCPP_INFO(this->get_logger()," Added priority order '%s' to open orders", order.id.c_str());
    }
    else{
        orders_.push_back(order);
        total_orders += 1;
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
        for(int i = 0; i<int(msg->bins.size()); i++){
            for( int j = 0; j< int(msg->bins[i].parts.size()); j++){
                CompetitorControlSystem::bin_set_quantity_for_this_part(msg->bins[i].bin_number, msg->bins[i].parts[j].part.type, msg->bins[i].parts[j].part.color, msg->bins[i].parts[j].quantity);
                RCLCPP_INFO(this->get_logger(),"Added Bin Part Quantity: %d", CompetitorControlSystem::bin_get_quantity_for_this_part(msg->bins[i].bin_number, msg->bins[i].parts[j].part.type, msg->bins[i].parts[j].part.color));
                RCLCPP_INFO_STREAM(get_logger(),"Added Bin Part type:"<< PartTypetoString( msg->bins[i].parts[j].part.type));
                RCLCPP_INFO_STREAM(get_logger(),"Added Bin Part Colour:"<< PartColortoString( msg->bins[i].parts[j].part.color));
            }
        }
        bin_read++;
    }
}

void CompetitorControlSystem::conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg){
    // This if statement is used to read the the conveyor topic only once. To be removed iN RWA3. 
    if(conveyor_read == 0){
        // add variable to update values here. 
        RCLCPP_INFO_STREAM(get_logger(),"Reading Conveyor Part "<< PartTypetoString(msg->parts[0].part.type));
        RCLCPP_INFO_STREAM(get_logger(),"Reading Conveyor Part "<< PartTypetoString(msg->parts[1].part.type));
        for( int i = 0; i<int(msg->parts.size()); i++){
            CompetitorControlSystem::conveyor_set_quantity_for_this_part(msg->parts[i].part.type, msg->parts[i].part.color, msg->parts[i].quantity);
            RCLCPP_INFO(this->get_logger(),"Added Conveyor Part Quantity: %d", msg->parts[i].quantity);
            RCLCPP_INFO_STREAM(get_logger(),"Added Conveyor Part type:"<< PartTypetoString(msg->parts[i].part.type));
            RCLCPP_INFO_STREAM(get_logger(),"Added Conveyor Part Colour:"<< PartColortoString( msg->parts[i].part.color));
            }
        conveyor_read++;
    }
    
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

void CompetitorControlSystem::ending_competition_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
    if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE){
        if (total_orders == 0){
            // Service client to End competition.
            RCLCPP_INFO(this->get_logger(),"Ending Competition");
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
            client = this->create_client<std_srvs::srv::Trigger>("/ariac/end_competition");
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result =client->async_send_request(request);
            if(result.get()->success)
                RCLCPP_INFO(this->get_logger(),"Competition Ended");
        }
    }
}

bool CompetitorControlSystem::InsufficientPartsChallange(OrderData current_order_){
    
    if (current_order_.type == ariac_msgs::msg::Order::KITTING){
        RCLCPP_INFO_STREAM(get_logger(),"Checking insufficient parts challange for Kitting task.");
        int insuf_part_kitting=4;
        for (auto j = 0; j < current_order_.kitting.kitting_parts.number_of_parts; j ++){
            for (int i =0; i<8; i++){
                if (bin_dictionary[i][current_order_.kitting.kitting_parts.parts_[j].type][current_order_.kitting.kitting_parts.parts_[j].color] > 0 && insuf_part_kitting!= 0){
                    kitting_part_details[current_order_.kitting.kitting_parts.parts_[j].quadrant].first = std::make_pair(current_order_.kitting.kitting_parts.parts_[j].type,current_order_.kitting.kitting_parts.parts_[j].color);
                    kitting_part_details[current_order_.kitting.kitting_parts.parts_[j].quadrant].second = i;
                    bin_set_quantity_for_this_part(i, current_order_.kitting.kitting_parts.parts_[j].type, current_order_.kitting.kitting_parts.parts_[j].color, -1);
                    insuf_part_kitting --;
                }
            }
            if (insuf_part_kitting != 0){
                if (conveyor_dictionary[current_order_.kitting.kitting_parts.parts_[j].type][current_order_.kitting.kitting_parts.parts_[j].color] >=1){
                    kitting_part_details[current_order_.kitting.kitting_parts.parts_[j].quadrant].first = std::make_pair(current_order_.kitting.kitting_parts.parts_[j].type,current_order_.kitting.kitting_parts.parts_[j].color);
                    kitting_part_details[current_order_.kitting.kitting_parts.parts_[j].quadrant].second = 9;
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
            RCLCPP_INFO_STREAM(get_logger()," Sufficient Parts available to complete Kitting Order.");
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
                    kitting_part_details[insuf_part_kitting].second = i;
                    bin_set_quantity_for_this_part(i, current_order_.combined.combined_parts.parts_[j].type, current_order_.combined.combined_parts.parts_[j].color, -1);
                    insuf_part_kitting --;
                    }
            }
            if (insuf_part_kitting != 0){
                if (conveyor_dictionary[current_order_.combined.combined_parts.parts_[j].type][current_order_.combined.combined_parts.parts_[j].color] >=1){
                    kitting_part_details[insuf_part_kitting].first = std::make_pair(current_order_.combined.combined_parts.parts_[j].type,current_order_.combined.combined_parts.parts_[j].color);
                    kitting_part_details[insuf_part_kitting].second = 9;
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

void CompetitorControlSystem::FloorRobotSendHome(){
    RCLCPP_INFO_STREAM(get_logger()," Moving Floor Robot to Home position");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
}

bool CompetitorControlSystem::FloorRobotPickandPlaceTray(int tray_id, uint8_t agv_no){
    // Checking for parts in bins/tables using camera
    RCLCPP_INFO (this->get_logger()," Checking Tray Tables for Tray Id: '%d' ", tray_id);
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    // Check if kit tray is on one of the two tables
    // geometry_msgs::msg::Pose tray_pose;
    std::string station;
    bool found_tray = false;
    RCLCPP_INFO(this->get_logger()," Moving Robot to Tray Table");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO (this->get_logger()," Checking if the Floor Robot has Tray gripper");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    if (floor_gripper_state_.type != "tray_gripper") {
        FloorRobotChangeGripper(station, "trays");
    }
    else {
        RCLCPP_INFO(this->get_logger()," Floor Robot gripper is correct");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    }
    RCLCPP_INFO(this->get_logger()," Moving to pickup tray ID: '%d'", tray_id);   
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO(this->get_logger()," Placing tray on AGV ID: '%d'", agv_no);
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
}

void CompetitorControlSystem::FloorRobotChangeGripper(std::string station, std::string gripper_type){
    RCLCPP_INFO(this->get_logger()," Changing Floor Robot Gripper to type '%s'", gripper_type.c_str() );
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
}

bool CompetitorControlSystem::FloorRobotPickBinPart(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part){
    uint8_t part_type = part.first.first;
    uint8_t part_color = part.first.second;
    uint8_t part_location = part.second;

    RCLCPP_INFO_STREAM(get_logger()," Reading Bins status, Part found in Bin : '"<< (std::to_string(part_location))<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");

    RCLCPP_INFO_STREAM(get_logger()," Attempting to pick a '" <<PartColortoString(part_color) << "' '" << PartTypetoString(part_type)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger()," Pickup part from the Bin Id: '"<< std::to_string(part_location)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
}

void CompetitorControlSystem::FloorRobotPlacePartOnKitTray(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part,int tray_id, uint8_t agv_no ){
    uint8_t part_type = part.first.first;
    uint8_t part_color = part.first.second;
    uint8_t part_location = part.second;
    RCLCPP_INFO_STREAM(get_logger()," Move robot to the AGV number : '"<< (std::to_string(agv_no))<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    
    RCLCPP_INFO_STREAM(get_logger()," Place '"<< PartColortoString(part_color)<<"' '"<<PartTypetoString(part_type)<<"' in quadrant '"<<std::to_string(quadrant)<< "' on the kit tray ID: '"<< std::to_string(tray_id)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
}

void CompetitorControlSystem::MoveAGVkitting(uint8_t agv, uint8_t destination){
    RCLCPP_INFO_STREAM(get_logger()," Locking Tray on AGV : '"<< std::to_string(agv)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger()," Moving AGV '"<< std::to_string(agv)<<"' to '"<< DestinationtoString(destination)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
}

void CompetitorControlSystem::MoveAGVAsComb(uint8_t agv, uint8_t station){
    RCLCPP_INFO_STREAM(get_logger()," Locking Tray on AGV "<< std::to_string(agv));
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger()," Moving AGV "<< std::to_string(agv)<<"to "<< StationtoString(station));
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
}

void CompetitorControlSystem::CeilingRobotSendHome(){
    RCLCPP_INFO_STREAM(get_logger()," Moving Ceiling Robot to Home position");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
}

void CompetitorControlSystem::CeilingRobotPickTrayPart(AssemblyInfo task){
    
    uint8_t part_location = task.station;
        RCLCPP_INFO_STREAM(get_logger()," Moving AGV '"<< std::to_string(task.station)<<"' to station: '"<<StationtoString(task.station)<<"'");
        if(task.agv_numbers.size() == 2){
        for (int j = 0; j < 2; j++){
            for (int i = 1; i <3; i++){
        uint8_t part_type = assembly_part_details[task.agv_numbers[j]][i].first;
        uint8_t part_color = assembly_part_details[task.agv_numbers[j]][i].second;
        RCLCPP_INFO_STREAM(get_logger()," Checking AGV for '"<<PartColortoString(part_color)<<"' '"<<PartTypetoString(part_type)<< "' location in tray ");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        RCLCPP_INFO_STREAM(get_logger()," Picking up '" <<PartColortoString(part_color) << "' '" << PartTypetoString(part_type)<<"'");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        RCLCPP_INFO_STREAM(get_logger()," Placing '"<<PartColortoString(part_color) << "' '" << PartTypetoString(part_type) << "' in Insert frame as per given pose and direction");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        }
        }
        }

        else if (task.agv_numbers.size() == 1){
        for (int i = 1; i <5; i++){
        uint8_t part_type = assembly_part_details[task.agv_numbers[0]][i].first;
        uint8_t part_color = assembly_part_details[task.agv_numbers[0]][i].second;
        RCLCPP_INFO_STREAM(get_logger()," Checking AGV for '"<<PartColortoString(part_color)<<"' '"<<PartTypetoString(part_type)<< "' location in tray ");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        RCLCPP_INFO_STREAM(get_logger()," Picking up '" <<PartColortoString(part_color) << "' '" << PartTypetoString(part_type)<<"'");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        RCLCPP_INFO_STREAM(get_logger()," Placing '"<<PartColortoString(part_color) << "' '" << PartTypetoString(part_type) << "' in Insert frame as per given pose and direction");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        }
        }
}

void CompetitorControlSystem::CeilingRobotPickTrayPart(CombinedInfo task){
    
    uint8_t part_location = task.station;
    CombinedTaskAssemblyUpdate(task);
        for (int i = 1; i <5; i++){
        uint8_t part_type = assembly_part_details[task.station][i].first;
        uint8_t part_color = assembly_part_details[task.station][i].second;
        RCLCPP_INFO_STREAM(get_logger()," Checking AGV for '"<<PartColortoString(part_color)<<"' '"<<PartTypetoString(part_type)<< "' location in tray ");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        RCLCPP_INFO_STREAM(get_logger()," Picking up '" <<PartColortoString(part_color) << "' '" << PartTypetoString(part_type)<<"'");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        RCLCPP_INFO_STREAM(get_logger()," Placing '"<<PartColortoString(part_color) << "' '" << PartTypetoString(part_type) << "' in Insert frame as per given pose and direction");
        RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
        }
        }

bool CompetitorControlSystem::CompleteKittingTask(KittingInfo task)
{ 
    FloorRobotSendHome();
    FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

    std::string station;
    FloorRobotChangeGripper(station, "parts");
    // RCLCPP_INFO_STREAM(get_logger()," Changing gripper to Part Gripper ");
    // RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    // Call function to change Robot Gripper

    for (auto kit_part = kitting_part_details.begin(); kit_part != kitting_part_details.end(); kit_part++){
        if(kit_part->second.second != NULL){
            FloorRobotPickBinPart(kit_part->first, kit_part->second);
            FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second,task.tray_id, task.agv_number);
        }
    }
    MoveAGVkitting(task.agv_number, task.destination);
    return true;
}

bool CompetitorControlSystem::CompleteAssemblyTask(AssemblyInfo task)
{   
    for ( int j = 0; j <(task.agv_numbers.size()); j++){
        MoveAGVAsComb(task.agv_numbers[j], task.station);
    }
    CeilingRobotSendHome();
    CeilingRobotPickTrayPart(task);
    return true;
}

bool CompetitorControlSystem::CompleteCombinedTask(CombinedInfo task){

    FloorRobotSendHome();
    uint8_t agv = task.station;
    int tray = 1;

    RCLCPP_INFO_STREAM(get_logger()," Choosing AGV : '"<< std::to_string(agv) << "' and Tray ID: '"<< std::to_string(tray)<<"' for combined task");
    // Functions will be updated during RWA3
    // agv = AGVAvailable(task.station);
    // tray = TrayAvailable(task.station);

    FloorRobotSendHome();
    FloorRobotPickandPlaceTray(tray, agv);
    RCLCPP_INFO_STREAM(get_logger()," Changing gripper to Part Gripper ");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");

    for (auto kit_part = kitting_part_details.begin(); kit_part != kitting_part_details.end(); kit_part++){
        if(kit_part->second.second != NULL){
            FloorRobotPickBinPart(kit_part->first, kit_part->second);
            FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second,tray, agv);
        }   
    }
    RCLCPP_INFO_STREAM(get_logger()," Moving AGV to station : '" <<StationtoString(task.station)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");   
    
    CeilingRobotSendHome();
    CeilingRobotPickTrayPart(task);

    return true;
}

void CompetitorControlSystem::CompleteOrders(){
    // Wait for first order to be published and status of bins and conveyor is read.
    while (orders_.size() == 0) {}
    while (bin_read == 0) {}
    while (conveyor_read == 0) {}
    bool success;
    while (true) {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
        success = false;
        break;
        }

        if (orders_.size() == 0){
        if (competition_state_  != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
            // wait for more orders
            RCLCPP_INFO(get_logger(), " Waiting for orders...");
            while (orders_.size() == 0) {}
            } 
        else {
            RCLCPP_INFO(get_logger(), " Completed all orders");
            success = true;
            break;
            }
        }

        OrderData current_order_ = orders_.front();
        orders_.erase(orders_.begin());
        total_orders--; 

        // Insufficient Parts challange
        CompetitorControlSystem::InsufficientPartsChallange(current_order_);

        if (current_order_.type == ariac_msgs::msg::Order::KITTING) {
            CompetitorControlSystem::CompleteKittingTask(current_order_.kitting);
            // Submit order
            CompetitorControlSystem::SubmitOrder(current_order_.id);
        } 
        else if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY) {
            CompetitorControlSystem::CompleteAssemblyTask(current_order_.assembly);
            // Submit order
            CompetitorControlSystem::SubmitOrder(current_order_.id);
        }
        else if (current_order_.type == ariac_msgs::msg::Order::COMBINED) {
            CompetitorControlSystem::CompleteCombinedTask(current_order_.combined);
            // Submit order
            CompetitorControlSystem::SubmitOrder(current_order_.id);
        }
        }
    }

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto competitor_control_system = std::make_shared<CompetitorControlSystem>("CCS");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(competitor_control_system);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    competitor_control_system->CompleteOrders();
    rclcpp::shutdown();
}

