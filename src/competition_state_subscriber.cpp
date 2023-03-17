# include "competition_state_subscriber.hpp"
# include "order_class.hpp"
# include <std_srvs/srv/trigger.hpp>
# include <memory>
# include <vector>
# include <iostream>  

void CompetitionStateSubscriber::competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
    // Reading if competition state is READY
    competition_state_ = msg->competition_state
    if(msg->competition_state == ariac_msgs::msg::CompetitionState::READY){
        RCLCPP_INFO(this->get_logger(),"Reading CompetitionState READY :%d", msg->competition_state);
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
        client = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        RCLCPP_INFO(this->get_logger(),"Starting Competition! ");
        auto result =client->async_send_request(request);
    } 
}

void CompetitionStateSubscriber::order_callback(const ariac_msgs::msg::Order::SharedPtr msg){   
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

int CompetitionStateSubscriber::bin_get_quantity_for_this_part(int bin, int part, int color){
    if (bin_dictionary.count(bin) == 0 || bin_dictionary[bin].count(part) == 0 || bin_dictionary[bin][part].count(color) == 0) {
        throw std::out_of_range("One of the keys does not exist in the map");
    }
    return bin_dictionary[bin][part][color];
}

void CompetitionStateSubscriber::bin_set_quantity_for_this_part(int bin, int part, int color, int value) {
    bin_dictionary[part][color] += value;
}

void CompetitionStateSubscriber::conveyor_set_quantity_for_this_part(int part, int color, int value) {
    conveyor_dictionary[part][color] += value;
}

// Subscriber Callback for reading bin status
void CompetitionStateSubscriber::bin_status_callback(const ariac_msgs::msg::BinParts::ConstSharedPtr msg){
    if (bin_read == 0){
        for( int i = 0; i<int(msg->bins.size()); i++){
            for( int j = 0; j< int(msg->bins[i].parts.size()); j++){
                CompetitionStateSubscriber::set_quantity_for_this_part(msg->bins[i].bin_number, msg->bins[i].parts[j].part.type, msg->bins[i].parts[j].part.color, msg->bins[i].parts[j].quantity);
                RCLCPP_INFO(this->get_logger(),"Added Bin Part %d", CompetitionStateSubscriber::get_quantity_for_this_part(msg->bins[i].bin_number, msg->bins[i].parts[j].part.type, msg->bins[i].parts[j].part.color));
            }
        }
        bin_read++;
    }
}


// Subscriber Callback for reading conveyor status
void CompetitionStateSubscriber::conveyor_status_callback(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg){
    if(conveyor_read == 0{
        RCLCPP_INFO(this->get_logger(),"Reading Conveyor Part %s,  ", msg->parts[0].type.c_str());
    })
    
}

// void CompetitionStateSubscriber::order_submission_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
//     // Reading if all orders are announced
//     if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE){
//         RCLCPP_INFO(this->get_logger(),"Response CompetitionState ORDER_ANNOUNCEMENTS_DONE :'%d'", msg->competition_state);
//         // Submitting orders from Stored data
//         for(auto i = orders_.begin(); i < orders_.end(); i++){
//             RCLCPP_INFO(this->get_logger(),"Submission of Order Id:%s", i->id.c_str() );
//             rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
//             client = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
//             auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
//             request->order_id = i->id;
//             auto result = client->async_send_request(request);
//             total_orders--;
//         }
//     }
// }

bool CompetitionStateSubscriber::SubmitOrder(std::string order_id)
{
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  client = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;
  auto result = client->async_send_request(request);
  return result.get()->success;
}

void CompetitionStateSubscriber::ending_competition_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
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

void CompetitionStateSubscriber::InsufficientPartsChallange(){
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

    }

void CompetitionStateSubscriber::CompleteOrders(){
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

        CompetitionStateSubscriber::InsufficientPartsChallange()

        if (current_order_.type == ariac_msgs::msg::Order::KITTING) {
        CompetitionStateSubscriber::CompleteKittingTask(current_order_.kitting_task);
        // Submit order
        CompetitionStateSubscriber::SubmitOrder(current_order_.id);
        } else if (current_order_.type == ariac_msgs::msg::Order::ASSEMBLY) {
        CompetitionStateSubscriber::CompleteAssemblyTask(current_order_.assembly_task);
        // Submit order
        CompetitionStateSubscriber::SubmitOrder(current_order_.id);
        } else if (current_order_.type == ariac_msgs::msg::Order::COMBINED) {
        CompetitionStateSubscriber::CompleteCombinedTask(current_order_.combined_task);
        }
    }
}

void CompetitionStateSubscriber::floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  floor_gripper_state_ = *msg;
}

void CompetitionStateSubscriber::FlooorRobotSendHome(){
    RCLCPP_INFO("Moving Floor Robot to Home position")
}

bool CompetitionStateSubscriber::FloorRobotPickandPlaceTray(int tray_id, int agv_no){
    // Checking for parts in bins/tables using camera
    RCLCPP_INFO ("Checking Tray Tables for Tray Id: %d : ", tray_id);
    // Check if kit tray is on one of the two tables
    // geometry_msgs::msg::Pose tray_pose;
    std::string station;
    bool found_tray = false;
    RCLCPP_INFO(this->get_logger()," Moving Robot to Tray Table")
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

void CompetitionStateSubscriber::FloorRobotChangeGripper(std::string station, std::string gripper_type){
    RCLCPP_INFO(this->get_logger(),"Changing Floor Robot Gripper to type: '%s'");
}

bool CompetitionStateSubscriber::FloorRobotPickBinPart(){
    RCLCPP_INFO_STREAM(get_logger()," Reading Bins status to check for part in Bin");
    // Implement code to check both the bins status here to find the parts. Use the implemented dictionary.
    // print(" Reading Conveyor Status to check for part in Conveyor")
    // // Implement code to check Conveyor status to find the parts if not found here. Use the implemented dictionary.

    RCLCPP_INFO_STREAM(get_logger(), "Attempting to pick a " <<part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);
    RCLCPP_INFO_STREAM(get_logger()," Pickup part from the Bin Id: '%d', Quadrant: '%d' ", bin_id, quadrant);
    // Implement function to pickup part from the Bin Id
    RCLCPP_INFO_STREAM(get_logger()," Change gripper to Part Gripper ");
    // Call function to change Robot Gripper
}

void CompetitionStateSubscriber::FloorRobotPlacePartOnKitTray(){
    RCLCPP_INFO_STREAM(get_logger()," Move robot to the AGV number :'%s'". AGV_Id);
    RCLCPP_INFO_STREAM(get_logger()," Place part on the kit tray ID: '%s'", tray_id);
}

void CompetitionStateSubscriber::MoveAGV(){
    RCLCPP_INFO_STREAM(get_logger(),"Locking AGV ");
    RCLCPP_INFO_STREAM(get_logger(),"Moving AGV to Warehouse");
}

void CompetitionStateSubscriber::CeilingRobotSendHome(){
    RCLCPP_INFO("Moving Floor Robot to Home position")
}

void CompetitionStateSubscriber::CeilingRobotPickTrayPart(){
    RCLCPP_INFO_STREAM(get_logger()," Checking AGV %s(part) location in tray");
    RCLCPP_INFO_STREAM(get_logger(), "Moving to Pick up " <<part_colors_[part_to_pick.color] << " " << part_types_[part_to_pick.type]);
    RCLCPP_INFO_STREAM(get_logger()," Picked up part from Tray");
}

void CompetitionStateSubscriber::CeilingRobotPlacePartInInsert(){
    RCLCPP_INFO_STREAM(get_logger()," Placing part in Inser for %s ", insert_insert_type);
}

int CompetitionStateSubscriber::AGVAvailable(){
    RCLCPP_INFO_STREAM(get_logger(),'Selecting available AGV here.')
}

bool CompetitionStateSubscriber::CompleteKittingTask() // change type of receiving variable
{ 
    FloorRobotSendHome();
    FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);
    for (auto kit_part: task.parts){
        FloorRobotPickBinPart(kit_part.part);
        FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
    }
    MoveAGV(task.agv_number, task.destination);
    return true;
}

bool CompetitionStateSubscriber::CompleteAssemblyTask()
{   
    MoveAGV(task.agv_number, task.destination);
    CeilingRobotSendHome();
    for (auto assembly_part: task.part){
        CeilingRobotPickTrayPart(Insert_variables_here);
        CeilingRobotPlacePartInInsert(Insert_variables_here);
    }
    return true;
}

bool CompetitionStateSubscriber::CompleteCombinedTask(){
    FloorRobotSendHome();
    agv = AGVAvailable();
    FloorRobotPickandPlaceTray(task.tray_id, agv);
    for (auto kit_part: task.parts){
        FloorRobotPickBinPart(kit_part.part);
        FloorRobotPlacePartOnKitTray(task.agv_number, kit_part.quadrant);
    }
    MoveAGV(task.agv_number, task.destination);
    CeilingRobotSendHome();
    for (auto assembly_part: task.parts){
        CeilingRobotPickTrayPart(Insert_variables_here);
        CeilingRobotPlacePartInInsert(Insert_variables_here);
    }
    return true;
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto comp_state_subscriber = std::make_shared<CompetitionStateSubscriber>("CompetitionStateSubscriber");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(comp_state_subscriber);
    // node->StartCompetition();
    // executor.spin();
    std::thread([&executor]()
                { executor.spin(); })
        .detach();
    comp_state_subscriber->CompetitionStateSubscriber();

    comp_state_subscriber->CompleteOrders();


    // rclcpp::spin(comp_state_subscriber);
    rclcpp::shutdown();
}

