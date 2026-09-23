#include "bt-ros2-interface/bt_sub_node.hpp"



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

