# include "competition_state_subscriber.hpp"
# include "order_class.hpp"
# include <std_srvs/srv/trigger.hpp>
# include <memory>
# include <vector>
# include <iostream>  

std::string CompetitorControlSystem::DestinationtoString(int dest){
    if (dest == 1)
        return "Assembly Front";
    else if (dest == 2)
        return "Assembly Back";
    else if (dest == 2)
        return "Warehouse";
    else
        return "unknown";
}

std::string CompetitorControlSystem::PartTypetoString(int part_type){
    if (part_type == 10)
        return "battery";
    else if (part_type == 11)
        return "pump";
    else if (part_type == 13)
        return "regulator";
    else if (part_type == 12)
        return "sensor";
    else
        return "unknown";
}

std::string CompetitorControlSystem::PartColortoString(int part_color){
    if (part_color == 0)
        return "red";
    else if (part_color == 1)
        return "green";
    else if (part_color == 2)
        return "blue";
    else if (part_color == 4)
        return "purple";
    else if (part_color == 3)
        return "orange";
    else
        return "unknown";
}

void CompetitorControlSystem::competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
    // Reading if competition state is READY
    competition_state_ = msg->competition_state;
    if(msg->competition_state == ariac_msgs::msg::CompetitionState::READY){
        RCLCPP_INFO(this->get_logger(),"Reading CompetitionState READY :%d", msg->competition_state);
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
        client = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        RCLCPP_INFO(this->get_logger(),"Starting Competition! ");
        auto result =client->async_send_request(request);
    } 
}

void CompetitorControlSystem::order_callback(const ariac_msgs::msg::Order::SharedPtr msg){   
    RCLCPP_INFO_STREAM(this->get_logger(), "Received order Id: " << msg->id);
    // Creating object of order_data to store current order
    OrderData order(msg); 
    //Storing order based on priority
    if(order.priority == true){
        orders_.insert(orders_.begin(),order);
        first_priority_order+=1;
        total_orders +=1; 
        RCLCPP_INFO(this->get_logger(),"Added priority order '%s' to open orders", order.id.c_str());
    }
    else{
        orders_.push_back(order);
        total_orders += 1;
        RCLCPP_INFO(this->get_logger(),"Added normal order '%s' to open orders", order.id.c_str());
    }
}

int CompetitorControlSystem::bin_get_quantity_for_this_part(int bin, int part, int color){
    if (bin_dictionary.count(bin) == 0 || bin_dictionary[bin].count(part) == 0 || bin_dictionary[bin][part].count(color) == 0) {
        throw std::out_of_range("One of the keys does not exist in the map");
    }
    return bin_dictionary[bin][part][color];
}

void CompetitorControlSystem::bin_set_quantity_for_this_part(int bin, int part, int color, int value) {
    bin_dictionary[part][color] += value;
}

void CompetitorControlSystem::conveyor_set_quantity_for_this_part(int part, int color, int value) {
    conveyor_dictionary[part][color] += value;
}

void CompetitorControlSystem::bin_status_callback(const ariac_msgs::msg::BinParts::ConstSharedPtr msg){
    if (bin_read == 0){
        for( int i = 0; i<int(msg->bins.size()); i++){
            for( int j = 0; j< int(msg->bins[i].parts.size()); j++){
                CompetitorControlSystem::bin_set_quantity_for_this_part(msg->bins[i].bin_number, msg->bins[i].parts[j].part.type, msg->bins[i].parts[j].part.color, msg->bins[i].parts[j].quantity);
                RCLCPP_INFO(this->get_logger(),"Added Bin Part %d", CompetitorControlSystem::bin_get_quantity_for_this_part(msg->bins[i].bin_number, msg->bins[i].parts[j].part.type, msg->bins[i].parts[j].part.color));
            }
        }
        bin_read++;
    }
}

void CompetitorControlSystem::conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg){
    if(conveyor_read == 0){
        // add variable to update values here. 
        RCLCPP_INFO(this->get_logger(),"Reading Conveyor Part %s, ", msg->parts[0].type.c_str());
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

void CompetitorControlSystem::InsufficientPartsChallange(){
    // for kitting task
    int insuf_part_kitting=4;
    for (auto part:current_order_.kitting_task.parts){
        for (int i =0, i<9, i++){
           if (bin_dictionary[i][part->part.type][part->part.color] >=1 && insuf_part_kitting!= 0){
                kitting_part_details[part.quandrant].first == std::make_pair(part->part.type,part->part.color);
                kitting_part_details[part.quandrant].second = i;
                bin_set_quantity_for_this_part(i, part->part.type, part->part.color, -1);
                insuf_part_kitting --;
                }
        }
        if (insuf_part_kitting != 0){
            if (conveyor_dictionary[part->part.type][part->part.color] >=1){
                kitting_part_details[part.quandrant].first == std::make_pair(part->part.type,part->part.color);
                kitting_part_details[part.quandrant].second = 9;
                conveyor_set_quantity_for_this_part(part->part.type, part->part.color, -1);
                insuf_part_kitting --;
                }
        }
        }
    
    // for assenbly task
    int insuf_part_assembly = 4;
  
    // for assembly task
    if(sizeof(current_order_.assembly_task.agv_numbers) == 2 && insuf_part_assembly != 0){

        int j = 0;
        for(auto it = assembly_part_details.begin(); it != assembly_part_details.end() && j <2 ; it ++, j++)
        {
            it->first = current_order_.assembly_task.agv_numbers[j];
            for (int i = 1; i <3; i++ ){
             it->second[i] = std::make_pair(part.type, part.color);
             insuf_part_assembly--;
            }
            
        }
    } else if (sizeof(current_order_.assembly.agv_numbers) == 1 && insuf_part_assembly != 0){ 
        assembly_part_details.begin()->first = current_order_.assembly_task.agv_numbers[0];
        for (int i = 1; i <5; i++){
            assembly_part_details.begin()->second[i] = std::make_pair(part.type, part.color);
            insuf_part_assembly--;
        }
    }
}

void CompetitorControlSystem::floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  floor_gripper_state_ = *msg;
}

void CompetitorControlSystem::FlooorRobotSendHome(){
    RCLCPP_INFO_STREAM(get_logger(),"Moving Floor Robot to Home position")
}

bool CompetitorControlSystem::FloorRobotPickandPlaceTray(int tray_id, int agv_no){
    // Checking for parts in bins/tables using camera
    RCLCPP_INFO (this->get_logger(),"Checking Tray Tables for Tray Id: %d : ", tray_id);
    // Check if kit tray is on one of the two tables
    // geometry_msgs::msg::Pose tray_pose;
    std::string station;
    bool found_tray = false;
    RCLCPP_INFO(this->get_logger()," Moving Robot to Tray Table");
    RCLCPP_INFO (this->get_logger(),"Checking if the Floor Robot has Tray gripper: ");
    if (floor_gripper_state_.type != "tray_gripper") {
        FloorRobotChangeGripper(station, "trays");
    }
    else {
        RCLCPP_INFO(this->get_logger(),"Floor Robot gripper is correct.");
    }
    RCLCPP_INFO(this->get_logger(),"Moving to pickup tray ID: %d", tray_id);   
    RCLCPP_INFO(this->get_logger(),"Placing tray on AGV ID: %d", agv_no);
}

void CompetitorControlSystem::FloorRobotChangeGripper(std::string station, std::string gripper_type){
    RCLCPP_INFO(this->get_logger(),"Changing Floor Robot Gripper " );
}

bool CompetitorControlSystem::FloorRobotPickBinPart(int quadrant, std::pair<std::pair<int, int>, int> part){
    int part_type = part.first.first;
    int part_color = part.first.second;
    int part_location = part.second;
    RCLCPP_INFO_STREAM(get_logger()," Reading Bins status, Part found in Bin "<< (std::to_string(part_location)));
    // Implement code to check both the bins status here to find the parts. Use the implemented dictionary.
    // print(" Reading Conveyor Status to check for part in Conveyor")
    // // Implement code to check Conveyor status to find the parts if not found here. Use the implemented dictionary.

    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " <<PartColortoString(part_color) << " " << PartTypetoString(part_type));
    RCLCPP_INFO_STREAM(get_logger()," Pickup part from the Bin Id: "<< std::to_string(part_location));
    // Implement function to pickup part from the Bin Id
    RCLCPP_INFO_STREAM(get_logger()," Changing gripper to Part Gripper ");
    // Call function to change Robot Gripper
}

void CompetitorControlSystem::FloorRobotPlacePartOnKitTray(int quadrant, std::pair<std::pair<int, int>, int> part,int tray_id, int agv_no ){
    int part_type = part.first.first;
    int part_color = part.first.second;
    int part_location = part.second;
    RCLCPP_INFO_STREAM(get_logger()," Move robot to the AGV number :"<< (std::to_string(agv_no)));
    RCLCPP_INFO_STREAM(get_logger()," Place"<< PartColortoString(part_color)<<" "<<PartTypetoString(part_type)<<" in quadrant "<<std::to_string(quadrant)<< "on the kit tray ID:"<< std::to_string(tray_id));
}

void CompetitorControlSystem::MoveAGV(int agv, int destination){
    RCLCPP_INFO_STREAM(get_logger(),"Locking Tray on AGV "<< std::to_string(agv));
    RCLCPP_INFO_STREAM(get_logger(),"Moving AGV to "<< std::to_string(destination));
}

void CompetitorControlSystem::CeilingRobotSendHome(){
    RCLCPP_INFO_STREAM(get_logger(),"Moving Ceiling Robot to Home position");
}

void CompetitorControlSystem::CeilingRobotPickTrayPart(int quadrant, std::pair<std::pair<int, int>, int> part){
    int part_type = part.first.first;
    int part_color = part.first.second;
    int part_location = part.second;
    RCLCPP_INFO_STREAM(get_logger()," Checking AGV "<<PartColortoString(part_color)<< PartTypetoString(part_type)<< "location in tray ");
    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick " <<PartColortoString(part_color) << " " << PartTypetoString(part_type));
    // RCLCPP_INFO_STREAM(get_logger()," Picked up part from Tray");
}

void CompetitorControlSystem::CeilingRobotPlacePartInInsert(){
    RCLCPP_INFO_STREAM(get_logger()," Placing part in Inser for ");
}

int CompetitorControlSystem::AGVAvailable(){
    RCLCPP_INFO_STREAM(get_logger(),"Selecting available AGV here.");
    return 0;
}

bool CompetitorControlSystem::CompleteKittingTask(KittingInfo task) // change type of receiving variable
{ 
    FloorRobotSendHome();
    FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);
    for (auto kit_part = kitting_part_details.begin(); kit_part != kitting_part_details.end(); kit_part++){
        FloorRobotPickBinPart(kit_part->first, kit_part->second);
        FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second,task.tray_id, task.agv_number);
    }
    MoveAGV(task.agv_number, task.destination);
    return true;
}

// bool CompetitorControlSystem::CompleteAssemblyTask()
// {   
//     MoveAGV(task.agv_number, task.destination);
//     CeilingRobotSendHome();
//     for (auto assembly_part: task.part){
//         CeilingRobotPickTrayPart(Insert_variables_here);
//         CeilingRobotPlacePartInInsert(Insert_variables_here);
//     }
//     return true;
// }

// bool CompetitorControlSystem::CompleteCombinedTask(){
//     FloorRobotSendHome();
//     agv = AGVAvailable();
//     FloorRobotSendHome();
//     FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);
//     for (auto kit_part = kitting_part_details.begin(), kit_part != kitting_part_details.end(), kit_part++){
//         FloorRobotPickBinPart(kit_part->first, kit_part->second);
//         FloorRobotPlacePartOnKitTray(kit_part->first, kit_part->second,task.tray_id, task.agv_number);
//     }
//     MoveAGV(task.agv_number, task.destination);
//     CeilingRobotSendHome();
//     for (auto assembly_part: task.parts){
//         CeilingRobotPickTrayPart(Insert_variables_here);
//         CeilingRobotPlacePartInInsert(Insert_variables_here);
//     }
//     return true;
// }

void CompetitorControlSystem::CompleteOrders(){
    // Wait for first order to be published
    while (orders_.size() == 0) {}

    bool success;
    while (true) {
        if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
        success = false;
        break;
        }

        if (orders_.size() == 0){
        if (competition_state_  != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
            // wait for more orders
            RCLCPP_INFO(get_logger(), "Waiting for orders...");
            while (orders_.size() == 0) {}
            } 
        else {
            RCLCPP_INFO(get_logger(), "Completed all orders");
            success = true;
            break;
            }
        }

        current_order_ = orders_.front();
        orders_.erase(orders_.begin());

        // CompetitorControlSystem::InsufficientPartsChallange()

        if (current_order_.type == ariac_msgs::msg::Order::KITTING) {
        CompetitorControlSystem::CompleteKittingTask(current_order_.kitting);
        // Submit order
        CompetitorControlSystem::SubmitOrder(current_order_.id);
        // } else if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY) {
        // CompetitorControlSystem::CompleteAssemblyTask(current_order_.assembly);
        // // Submit order
        // CompetitorControlSystem::SubmitOrder(current_order_.id);
        // } else if (current_order_.type == ariac_msgs::msg::Order::COMBINED) {
        // CompetitorControlSystem::CompleteCombinedTask(current_order_.combined);
        // Submit order
        // CompetitorControlSystem::SubmitOrder(current_order_.id);
        // }
        }
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto competitor_control_system = std::make_shared<CompetitorControlSystem>("CCS");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(competitor_control_system);
    // node->StartCompetition();
    // executor.spin();
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    // competitor_control_system->CompetitorControlSystem();

    competitor_control_system->CompleteOrders();

    // rclcpp::spin(comp_state_subscriber);
    rclcpp::shutdown();
}

