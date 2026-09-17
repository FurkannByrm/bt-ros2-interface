#include "bt-ros2-interface/bt_sub_node.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <std_msgs/msg/detail/bool__struct.hpp>



WaitForSignal::WaitForSignal(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode{name,config} {}


BT::NodeStatus WaitForSignal::onStart(){ 
        start_time_ = std::chrono::steady_clock::now();
        
        return checkCondition();
}

BT::NodeStatus WaitForSignal::onRunning(){

        BT::Expected<double> timeout_sec = getInput<double>("timeout");
      
       if (!timeout_sec) {
        return BT::NodeStatus::FAILURE;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time_).count();
        if(timeout_sec.value() > 0.0 && elapsed > timeout_sec.value()){ 
        return BT::NodeStatus::FAILURE; 
        }
        
        return checkCondition();
}

BT::NodeStatus WaitForSignal::checkCondition(){
    auto expected_val = getInput<bool>("expected_value");
    auto current_val  = getInput<bool>("topic");

        if(!expected_val || !current_val){
            return BT::NodeStatus::FAILURE;
        }

       if(expected_val.value() == current_val.value()){
           return  BT::NodeStatus::SUCCESS;
       } 

       return BT::NodeStatus::RUNNING; 

}



CarLocatedNode::CarLocatedNode(const std::string& node_name, BT::Blackboard::Ptr blackboard) : Node{node_name}, blackboard_{blackboard}{
    blackboard_->set<bool>("is_car_body_located", false);

   carbody_located_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/car_located", 10,
    [list = this->blackboard_](const std_msgs::msg::Bool::SharedPtr msg) {
        list->set("is_car_body_located", msg->data);
    }
);



}

CheckState::CheckState(const std::string& node_name, const BT::NodeConfig& config) : BT::SyncActionNode(node_name,config){}


BT::NodeStatus CheckState::tick(){

    auto home_status = getInput<bool>("topic");
    if (!home_status) {
        return BT::NodeStatus::FAILURE;
    }

    if(home_status.value()){
        return BT::NodeStatus::SUCCESS;
    }
    else{
        return BT::NodeStatus::FAILURE;
    }


}

