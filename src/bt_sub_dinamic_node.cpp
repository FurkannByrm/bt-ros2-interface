#include <behaviortree_cpp/bt_factory.h>
#include <bt-ros2-interface/bt_sub_dinamic_node.hpp>

SubNodeBridge::SubNodeBridge(const std::string& node_name) : node_{std::make_shared<rclcpp::Node>(node_name)}{}
void SubNodeBridge::subscribeIfNeeded(const std::string& topic){
        
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub;
    bool map_check =false;

    {
        std::lock_guard<std::mutex> lock(mtx_);  
        if(subs_map_.find(topic) == subs_map_.end()){
        map_check = true;
        }

    }
           
        if(map_check){
         RCLCPP_INFO(node_->get_logger(), "Subscriber is not founded, it is being creating...");
          sub = node_->create_subscription<std_msgs::msg::Bool>(topic,rclcpp::QoS(20).best_effort(),[this,topic](const std_msgs::msg::Bool::ConstSharedPtr& msg ){                                                
          std::lock_guard<std::mutex> lock(mtx_);
          subs_map_[topic].second = msg->data;
          });

        
        {
          std::lock_guard<std::mutex> lock(mtx_);
          subs_map_[topic].first = sub;
        }

        }


}

bool SubNodeBridge::consumeIfMatches(const std::string& topic, bool exp_val){ 
            std::lock_guard<std::mutex> lock2(mtx_); 
            if(subs_map_[topic].second == exp_val){
                RCLCPP_INFO(node_->get_logger(),"Match is complete");
                subs_map_[topic].second.reset();
                return true;
}else{
                return false;
            }

}

WaitNodeBT::WaitNodeBT(const std::string& node_name, const BT::NodeConfig& config) : BT::StatefulActionNode{node_name, config}{


    if (!getInput("topic", topic_name_)) {
            throw BT::RuntimeError("there is no port: topic");
        }
        if (!getInput("expected_value", expect_val_)) {
            throw BT::RuntimeError("there is no port: expected_value");
        }
        getInput("timeout", timeout_duration_);

}
BT::NodeStatus WaitNodeBT::onStart(){

        if (!config().blackboard->get("ros_subscriber_bridge", bridge_)) {
            throw BT::RuntimeError("'ros_bridge' not founded!");
        }
        bridge_->subscribeIfNeeded(topic_name_); 
        start_time_ =std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;

}

BT::NodeStatus WaitNodeBT::onRunning(){

auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time_).count();

    if(elapsed > timeout_duration_){
        return BT::NodeStatus::FAILURE;
    }

    if(bridge_->consumeIfMatches(topic_name_,expect_val_)){
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::RUNNING;

}








