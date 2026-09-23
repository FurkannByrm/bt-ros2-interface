#ifndef BT_SERVICE_NODE_HPP_
#define BT_SERVICE_NODE_HPP_
#include <behaviortree_cpp/blackboard.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <std_srvs/srv/set_bool.hpp>

class ClientReqBridge{

    public:
        ClientReqBridge(const std::string& node_name = "demonstrator_client_node"); 
    
        rclcpp::Node::SharedPtr getNode()const{
            return node_;
        }

        bool sendRequest(const std::string& service_name,
                         bool payload,
                         rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture& out_future);
   
    private:
        std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> client_map_;
        rclcpp::Node::SharedPtr node_;
        std::mutex mutex_;

};


class ClientReqBT : public BT::StatefulActionNode{

    public:
        ClientReqBT(const std::string& node_name, const BT::NodeConfig& config);
        static BT::PortsList providedPorts(){

            return  {
            BT::InputPort<std::string>("topic", "target service name"),
            BT::InputPort<bool>("request_payload", "sending value"),
            BT::InputPort<double>("timeout", 3.0, "Max timeout sn")
            };
        }

        BT::NodeStatus onStart()override;
        BT::NodeStatus onRunning()override;
        void onHalted()override{
            response_future_ = {};
        }

    private:
     
    std::shared_ptr<ClientReqBridge> bridge_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response_future_;
    std::string service_name_;
    bool payload_{false};
    double timeout_duration_{3.0};
    std::chrono::steady_clock::time_point start_time_;

};



#endif //BT_SERVICE_NODE_HPP_
       //


