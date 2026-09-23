#include <bt-ros2-interface/bt_sub_dinamic_node.hpp>
#include <bt-ros2-interface/bt_service_dinamic_node.hpp>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    auto node_sub = std::make_shared<SubNodeBridge>();
    auto node_client = std::make_shared<ClientReqBridge>();
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(node_sub->getNode());
    exe.add_node(node_client->getNode());
    std::thread sub_thread{[&exe](){
            exe.spin();
    }};


    auto blackboard = BT::Blackboard::create();
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<WaitNodeBT>("WaitForSignal");
    factory.registerNodeType<ClientReqBT>("ClientReq"); 
     blackboard->set("ros_client_bridge", node_client);
     blackboard->set("ros_subscriber_bridge", node_sub);
auto tree = factory.createTreeFromFile("/home/furkan/magician_ws/src/bt-ros2-interface/config/bt_dinamic_test.xml", blackboard);
 tree.tickWhileRunning();   

 rclcpp::shutdown();
 if(sub_thread.joinable()){

     sub_thread.join();
 }



    return 0;
}





