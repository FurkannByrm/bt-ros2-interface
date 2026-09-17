#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/bt_factory.h>
#include <bt-ros2-interface/bt_sub_node.hpp>
#include <memory>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <xbot_msgs/msg/detail/joint_state__struct.hpp>

int main(int argc, char* argv[]){

rclcpp::QoS qos(10);
    rclcpp::init(argc, argv);
      
    
    auto blackboard = BT::Blackboard::create();
    auto node_sub = std::make_shared<SubNode<std_msgs::msg::Bool>>("car_node",rclcpp::QoS(10),"/is_car_body_located",blackboard);
    auto sensing_homing_sub = std::make_shared<SubNode<xbot_msgs::msg::JointState>>("sensing_home_check_node",rclcpp::SensorDataQoS(),
    "/sr/xbotcore/joint_states",blackboard,  std::vector<double>{0.0045, 0.5, 1.5, 0.003, 0.99, 0.002});
    // auto node_sub = std::make_shared<CarLocatedNode>("carla",blackboard);
    auto cleaning_homing_sub = std::make_shared<SubNode<xbot_msgs::msg::JointState>>("cleaning_home_check_node",rclcpp::SensorDataQoS(),
    "/cr/xbotcore/joint_states",blackboard,  std::vector<double>{0.003,0.521,1.507,0.002,1.003,-0.002});

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(node_sub);
    exe.add_node(sensing_homing_sub);
    exe.add_node(cleaning_homing_sub);
    std::thread sub_thread([&exe]() {
        exe.spin();
    });


    RCLCPP_INFO(rclcpp::get_logger("Main"), "waiting for joint states..");

    bool sr_ready = sensing_homing_sub->waitForFirstMessage(std::chrono::seconds(5));
    bool cr_ready = cleaning_homing_sub->waitForFirstMessage(std::chrono::seconds(5));

    if (!sr_ready || !cr_ready) {
        RCLCPP_ERROR(rclcpp::get_logger("Main"), 
            "timeout", 
            sr_ready, cr_ready);
        
        exe.cancel();
        rclcpp::shutdown();
        if (sub_thread.joinable()) sub_thread.join();
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("Main"), "bt began.");

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<WaitForSignal>("WaitForSignal");
    factory.registerNodeType<CheckState>("CheckState");
auto tree = factory.createTreeFromFile("/home/plot/magician-demonstrator/src/bt-ros2-interface/config/bt_test.xml", blackboard);
 tree.tickWhileRunning();   
exe.cancel();
rclcpp::shutdown();
if(sub_thread.joinable()){
    sub_thread.join();
}

return 0;
}
