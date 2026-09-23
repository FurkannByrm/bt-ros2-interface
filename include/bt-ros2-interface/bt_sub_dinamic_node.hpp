#ifndef BT_SUB_DINAMIC_NODE_HPP_
#define BT_SUB_DINAMIC_NODE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <std_msgs/msg/bool.hpp>


class SubNodeBridge{

    public:
        SubNodeBridge(const std::string& node_name = "demonstrator_subscription_node");
       
        rclcpp::Node::SharedPtr getNode()const{
            return node_;
        }

    void subscribeIfNeeded(const std::string& topic);
    bool consumeIfMatches(const std::string& topic, bool exp_val);

    private:
        std::unordered_map<std::string,std::pair<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr,std::optional<bool>>> subs_map_;
        rclcpp::Node::SharedPtr node_;
        std::mutex mtx_;
        

};


class WaitNodeBT : public  BT::StatefulActionNode{

    public:
        WaitNodeBT(const std::string& node_name, const BT::NodeConfig& config);

        BT::NodeStatus onStart()override;
        BT::NodeStatus onRunning()override;
        void onHalted()override{}

        static BT::PortsList providedPorts(){
           return{
               BT::InputPort<std::string>("topic","target topic name"),
               BT::InputPort<bool>("expected_value","callback value"),
               BT::InputPort<double>("timeout","Waiting timeout")

           }; 
        }


    private:
        std::string topic_name_;
        std::shared_ptr<SubNodeBridge> bridge_;
        bool expect_val_{false};
        double timeout_duration_{5.0};
        std::chrono::steady_clock::time_point start_time_;
};






#endif
