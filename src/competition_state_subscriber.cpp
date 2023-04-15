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
  }

  left_bins_parts_ = msg->part_poses;
  left_bins_camera_pose_ = msg->sensor_pose;
}

void CompetitorControlSystem::right_bins_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg) 
{
  if (!right_bins_camera_recieved_data) {
    RCLCPP_INFO(this->get_logger(), "Received data from right bins camera");
    right_bins_camera_recieved_data = true;
  }

  right_bins_parts_ = msg->part_poses;
  right_bins_camera_pose_ = msg->sensor_pose;
}

void CompetitorControlSystem::conveyor_camera_cb(const ariac_msgs::msg::AdvancedLogicalCameraImage::ConstSharedPtr msg){
    if (!conveyor_camera_received_data && (msg->part_poses.size() != 0)) {
        RCLCPP_INFO(this->get_logger(), "Received data from conveyor camera");
        conveyor_camera_received_data = true;
    }
    conv_part_ = msg->part_poses;
    conv_camera_pose_ = msg->sensor_pose; 
}

void CompetitorControlSystem::breakbeam_start_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    
}

void CompetitorControlSystem::breakbeam_end_cb(const ariac_msgs::msg::BreakBeamStatus::ConstSharedPtr msg){
    if(!breakbeam_end_received_data && msg->object_detected == true){
        breakbeam_end_received_data = true;
    }
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

void CompetitorControlSystem::EndCompetition(){
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE){
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

// void CompetitorControlSystem::LogPose(geometry_msgs::msg::Pose p)
// {
//   tf2::Quaternion q(
//     p.orientation.x,
//     p.orientation.y,
//     p.orientation.z,
//     p.orientation.w);
//   tf2::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);

//   roll *= 180/M_PI;
//   pitch *= 180/M_PI;
//   yaw *= 180/M_PI;

//   RCLCPP_INFO(get_logger(), "(X: %.2f, Y: %.2f, Z: %.2f, R: %.2f, P: %.2f, Y: %.2f)",
//                  p.position.x, p.position.y, p.position.z,
//                  roll, pitch, yaw);
// }

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
    // Move floor robot to home joint state
    floor_robot_.setNamedTarget("home");
    FloorRobotMovetoTarget();
}

bool CompetitorControlSystem::LockAGVTray(int agv_num)
{
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

bool CompetitorControlSystem::FloorRobotPickBinPart(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part){
    uint8_t part_type = part.first.first;
    uint8_t part_color = part.first.second;
    uint8_t part_bin_location = part.second;

    RCLCPP_INFO_STREAM(get_logger()," Reading Bins status, Part found in Bin : '"<< (std::to_string(part_bin_location))<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    // Check if part is in one of the bins
    geometry_msgs::msg::Pose part_pose;
    bool found_part = false;
    std::string bin_side;

    // Check left bins
    for (auto part: left_bins_parts_) {
        if (part.part.type == part_type && part.part.color == part_color) {
        part_pose = MultiplyPose(left_bins_camera_pose_, part.pose);
        found_part = true;
        bin_side = "left_bins";
        break;
        }
    }
    // Check right bins
    if (!found_part) {
        for (auto part: right_bins_parts_) {
        if (part.part.type == part_type && part.part.color == part_color) {
            part_pose = MultiplyPose(right_bins_camera_pose_, part.pose);
            found_part = true;
            bin_side = "right_bins";
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
    RCLCPP_INFO_STREAM(get_logger(),"-----------------------------------ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py-----------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger()," Pickup part from the Bin Id: '"<< std::to_string(part_bin_location)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");

    floor_robot_.setJointValueTarget("linear_actuator_joint", rail_positions_[bin_side]);
    floor_robot_.setJointValueTarget("floor_shoulder_pan_joint", 0);
    FloorRobotMovetoTarget();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
        part_pose.position.z + 0.5, SetRobotOrientation(part_rotation)));
    
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
        part_pose.position.z + part_heights_[part_type] + pick_offset_, SetRobotOrientation(part_rotation)));
    
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    FloorRobotSetGripperState(true);

    FloorRobotWaitForAttach(3.0);

    // Add part to planning scene
    std::string part_name = part_colors_[part_color] + "_" + part_types_[part_type];
    AddModelToPlanningScene(part_name, part_types_[part_type] + ".stl", part_pose);
    floor_robot_.attachObject(part_name);
    // floor_robot_attached_part_ = part_to_pick;
    floor_robot_attached_part_.type = part_type;
    floor_robot_attached_part_.color = part_color;

    // Move up slightly
    waypoints.clear();
    waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y, 
        part_pose.position.z + 0.3, SetRobotOrientation(0)));

    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    return true;
}

bool CompetitorControlSystem::FloorRobotPlacePartOnKitTray(uint8_t quadrant, std::pair<std::pair<uint8_t, uint8_t>, uint8_t> part,int tray_id, uint8_t agv_no ){
    uint8_t part_type = part.first.first;
    uint8_t part_color = part.first.second;
    uint8_t part_location = part.second;
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
        part_drop_pose.position.z + part_heights_[floor_robot_attached_part_.type] + drop_height_,
        SetRobotOrientation(0)));
    
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);

    // Drop part in quadrant
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

bool CompetitorControlSystem::FloorRobotConveyorPartspickup(){
    floor_robot_.setJointValueTarget(floor_conveyor_parts_pickup);
    FloorRobotMovetoTarget();
    // FloorRobotSetGripperState(true);
    
    geometry_msgs::msg::Pose part_pose;
    bool found_part = false;
    
    
    uint8_t part_type;
    uint8_t part_color;

    while(!conveyor_camera_received_data){
        // RCLCPP_INFO(this->get_logger(), "Waiting for part on conveyor");
    }

    conv_part_current = conv_part_;
    part_pose = MultiplyPose(conv_camera_pose_, conv_part_current[0].pose);
    part_type = conv_part_current[0].part.type;
    part_color = conv_part_current[0].part.color;

    // double part_rotation = GetYaw(part_pose);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    // waypoints.push_back(BuildPose(part_pose.position.x, part_pose.position.y - 2.5, 
    //     part_pose.position.z+ 0.3, SetRobotOrientation(0)));

    waypoints.push_back(BuildPose(part_pose.position.x, 0.65 , 
        part_pose.position.z + part_heights_[part_type] + pick_offset_-0.001, SetRobotOrientation(0)));
    
    FloorRobotMoveCartesian(waypoints, 0.3, 0.3);
    
    FloorRobotSetGripperState(true);
    // function to move robot at 0.2m/s
    // function for robot to go down to pickup part(x, y, z) // x-from camera pose, y-changes with time, z-part height
    while (breakbeam_end_received_data== false){}
    breakbeam_end_received_data == false; 
    FloorRobotWaitForAttach(2.0); 


}

bool CompetitorControlSystem::MoveAGVkitting(uint8_t agv, uint8_t destination){
    RCLCPP_INFO_STREAM(get_logger()," Locking Tray on AGV : '"<< std::to_string(agv)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");
    RCLCPP_INFO_STREAM(get_logger()," Moving AGV '"<< std::to_string(agv)<<"' to '"<< DestinationtoString(destination)<<"'");
    RCLCPP_INFO_STREAM(get_logger(),"----------------------------------------------------------------------------------");

    rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

    std::string srv_name = "/ariac/move_agv" + std::to_string(agv);

    client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

    auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
    request->location = destination;

    auto result =client->async_send_request(request);
    result.wait();

    return result.get()->success;
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
    FloorRobotConveyorPartspickup();

    FloorRobotSendHome();
    FloorRobotPickandPlaceTray(task.tray_id, task.agv_number);

    std::string station;
    FloorRobotChangeGripper(station, "parts");

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
            CompetitorControlSystem::EndCompetition();
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

    competitor_control_system->FloorRobotSendHome();

    competitor_control_system->CompleteOrders();
    rclcpp::shutdown();
}

