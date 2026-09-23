#ifndef BT_SUB_NODE_HPP
#define BT_SUB_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <std_msgs/msg/bool.hpp>
#include <xbot_msgs/msg/joint_state.hpp>

class WaitForSignal : public  BT::StatefulActionNode{

    public:

    WaitForSignal(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts(){
    return {
            BT::InputPort<bool>("topic","blackboard value"),
            BT::InputPort<bool>("expected_value",true,"wanted value"),
            BT::InputPort<double>("timeout",30.0,"timeout") 
            };
    }

    BT::NodeStatus onStart()override;
    BT::NodeStatus onRunning()override;

    void onHalted() override{}

    private:
    std::chrono::steady_clock::time_point start_time_;
    BT::NodeStatus checkCondition();

 };

class CheckState : public BT::SyncActionNode{

    public:
    CheckState(const std::string& node_name, const BT::NodeConfig& config);
    
static BT::PortsList providedPorts(){

    return {BT::InputPort<bool>("topic","blackboard value")};

}
    BT::NodeStatus tick()override;

};


template<typename T>
class SubNode : public rclcpp::Node{
   using MessageTypeSharedPtr = typename T::SharedPtr;
   using MessageTypeConstSharedPtr = typename T::ConstSharedPtr; 


    public:
        SubNode(const std::string& node_name, rclcpp::QoS qos, const std::string& topic_name, BT::Blackboard::Ptr blackboard) : Node{node_name}, blackboard_{blackboard}{
            blackboard_->set(topic_name, false); 
            sub_ = this->create_subscription<T>(topic_name,qos, [list = this->blackboard_,topic_name](const MessageTypeSharedPtr msg){
           
                    list->set(topic_name,msg->data); });
        } 

    private:
       typename rclcpp::Subscription<T>::SharedPtr sub_;
        BT::Blackboard::Ptr blackboard_;
};

template<>
class SubNode<xbot_msgs::msg::JointState> : public rclcpp::Node {
public:
    SubNode(const std::string& node_name, 
            rclcpp::QoS qos, 
            const std::string& topic_name, 
            BT::Blackboard::Ptr blackboard, 
            const std::vector<double>& home_vec) 
        : Node{node_name}, 
          blackboard_{blackboard},
          first_msg_future_{first_msg_promise_.get_future()} 
    {
        homing_control_sub_ = this->create_subscription<xbot_msgs::msg::JointState>(
            topic_name, qos, 
            [this, home_vec, topic_name](const xbot_msgs::msg::JointState::ConstSharedPtr& msg) {
                
                bool is_home = true;
                for (size_t i = 0; i < home_vec.size() && i < msg->link_position.size(); ++i) {
                    if (std::fabs(home_vec[i] - msg->link_position[i]) > 0.01) {
                        is_home = false;
                        break;
                    }
                }

                blackboard_->set(topic_name, is_home);

                if (is_home) {
                    RCLCPP_INFO(this->get_logger(), "%s - ROBOT HOME POSITION", topic_name.c_str());
                } else {
                    RCLCPP_INFO(this->get_logger(), "%s - Robot NOT home", topic_name.c_str());
                }

                if (!signaled_.exchange(true)) {
                    first_msg_promise_.set_value();
                }
            });
    }

    bool waitForFirstMessage(std::chrono::seconds timeout = std::chrono::seconds(5))
    {
        return first_msg_future_.wait_for(timeout) == std::future_status::ready;
    }

private:
    rclcpp::Subscription<xbot_msgs::msg::JointState>::SharedPtr homing_control_sub_;
    BT::Blackboard::Ptr blackboard_;

    std::promise<void> first_msg_promise_;
    std::future<void> first_msg_future_;
    std::atomic<bool> signaled_{false};
};



#endif //BT_SUB_NODE_HPP

