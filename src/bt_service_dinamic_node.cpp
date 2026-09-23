#include <bt-ros2-interface/bt_service_dinamic_node.hpp>


ClientReqBridge::ClientReqBridge(const std::string& node_name) : node_{std::make_shared<rclcpp::Node>(node_name)}{   
}

bool ClientReqBridge::sendRequest(const std::string& service_name,
                                  bool payload,
                                  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture& out_future){

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client;

    {
        std::lock_guard<std::mutex> lock(mutex_);  
        auto it = client_map_.find(service_name);
        if(it == client_map_.end()){
           RCLCPP_INFO(node_->get_logger(), "client is not founded, it is being creating...");
           client = node_->create_client<std_srvs::srv::SetBool>(service_name);
           client_map_[service_name] = client;
        }
        else{
            client = it->second;
        }

    }
    
    if (!client->service_is_ready()) {
            if (!client->wait_for_service(std::chrono::milliseconds(50))) {
                RCLCPP_WARN(node_->get_logger(), "Servis is not ready: %s", service_name.c_str());
                return false;
            }
        }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = payload;

    out_future = client->async_send_request(request).share();
    return true;
}

ClientReqBT::ClientReqBT(const std::string& node_name, const BT::NodeConfig& config) : BT::StatefulActionNode{node_name, config}{}


BT::NodeStatus ClientReqBT::onStart(){

    if (!getInput("topic", service_name_)) {
            throw BT::RuntimeError("there is no port: topic");
        }
        if (!getInput("request_payload", payload_)) {
            throw BT::RuntimeError("there is no port: request_payload");
        }
        getInput("timeout", timeout_duration_);

        if (!config().blackboard->get("ros_client_bridge", bridge_)) {
            throw BT::RuntimeError("'ros_bridge' not founded!");
        }

        if (!bridge_->sendRequest(service_name_, payload_, response_future_)) {
            return BT::NodeStatus::FAILURE;
        }
    
        start_time_ =std::chrono::steady_clock::now();
        return BT::NodeStatus::RUNNING;

}

BT::NodeStatus ClientReqBT::onRunning(){

    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - start_time_).count();
        
if (elapsed > timeout_duration_) {
            return BT::NodeStatus::FAILURE;
        }

        if (response_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            auto response = response_future_.get();
            if (response && response->success) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;


}
