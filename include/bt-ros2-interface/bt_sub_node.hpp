#ifndef BT_SUB_NODE_HPP
#define BT_SUB_NODE_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/tree_node.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <xbot_msgs/msg/detail/joint_state__struct.hpp>
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
class SubNode<xbot_msgs::msg::JointState> : public rclcpp::Node{

    public:
        SubNode(const std::string& node_name, rclcpp::QoS qos, const std::string& topic_name, BT::Blackboard::Ptr blackboard, const std::vector<double>& home_vec) : Node{node_name}, blackboard_{blackboard}{
        blackboard_->set(topic_name,false);
        homing_control_sub_ = this->create_subscription<xbot_msgs::msg::JointState>(topic_name,qos,[this, home_vec, topic_name](const xbot_msgs::msg::JointState::ConstSharedPtr& msg){
        auto is_home = true;
    for(size_t i = 0; i<home_vec.size(); i++)
    {   
            auto target = home_vec[i];       
            auto current = msg->link_position[i];
            
            if(std::fabs(target - current) > 0.01 ){
                is_home = false;
                break;
            }
    }
 
      if (is_home){
        blackboard_->set(topic_name,true); 
        RCLCPP_INFO(this->get_logger(), "%s - ROBOT HOME POSITION", topic_name.c_str());
    }else
    {
        blackboard_->set(topic_name,false);
        RCLCPP_INFO(get_logger(), "%s - Robot NOT home", topic_name.c_str());
    }

    
    });

        }



    private:
    
    rclcpp::Subscription<xbot_msgs::msg::JointState>::SharedPtr homing_control_sub_;
    BT::Blackboard::Ptr blackboard_;

};




class CarLocatedNode : public rclcpp::Node{
    
    public:
    
        CarLocatedNode(const std::string& node_name,BT::Blackboard::Ptr blackboard);

    private:

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr carbody_located_sub_;
        

        BT::Blackboard::Ptr blackboard_;
};










#endif //BT_SUB_NODE_HPP

