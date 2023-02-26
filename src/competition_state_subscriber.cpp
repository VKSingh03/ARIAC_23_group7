# include "competition_state_subscriber.hpp"
# include "order_class.hpp"
# include <std_srvs/srv/trigger.hpp>
# include <memory>
# include <vector>
#include<iostream>  

void CompetitionStateSubscriber::competition_state_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
    // Reading if competition state is READY
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
        RCLCPP_INFO(this->get_logger(),"Added priority order %s", order.id.c_str());
    }
    else{
        orders_.push_back(order);
        total_orders += 1;
        RCLCPP_INFO(this->get_logger(),"Added normal order %s", order.id.c_str());
    }
}

void CompetitionStateSubscriber::order_submission_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
    // Reading if all orders are announced
    if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE){
        RCLCPP_INFO(this->get_logger(),"Response CompetitionState ORDER_ANNOUNCEMENTS_DONE :'%d'", msg->competition_state);
        // Submitting orders from Stored data
        for(auto i = orders_.begin(); i < orders_.end(); i++){
            RCLCPP_INFO(this->get_logger(),"Submission of Order Id:%s", i->id.c_str() );
            rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
            client = this->create_client<ariac_msgs::srv::SubmitOrder>("/ariac/submit_order");
            auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
            request->order_id = i->id;
            auto result = client->async_send_request(request);
            total_orders--;
        }
    }
}

void CompetitionStateSubscriber::ending_competition_callback(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg){
    if (msg->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE){
        if (total_orders == 0){
            // Service client to End competition.
            RCLCPP_INFO(this->get_logger(),"Ending Competition :");
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
            client = this->create_client<std_srvs::srv::Trigger>("/ariac/end_competition");
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto result =client->async_send_request(request);
            if(result.get()->success)
                RCLCPP_INFO(this->get_logger(),"Competition Ended");
        }
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto comp_state_subscriber = std::make_shared<CompetitionStateSubscriber>("CompetitionStateSubscriber");
    rclcpp::spin(comp_state_subscriber);
    rclcpp::shutdown();
}

