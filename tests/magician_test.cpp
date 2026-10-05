
#include <bt-ros2-interface/bt_sub_dinamic_node.hpp>
#include <bt-ros2-interface/bt_service_dinamic_node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>



int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    
    auto tmp_node = std::make_shared<rclcpp::Node>("bt_xml_loader");
    tmp_node->declare_parameter<std::string>("config","magician_demonstrator_without_robot.xml");
    std::string config_name;
    tmp_node->get_parameter("config",config_name);
    tmp_node.reset();
    
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("bt-ros2-interface");
    const std::string bt_tree_xml_file = pkg_share + "/config/" + config_name;

    RCLCPP_INFO(rclcpp::get_logger("main"), "Loading config: %s", bt_tree_xml_file.c_str());

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


auto tree = factory.createTreeFromFile(bt_tree_xml_file, blackboard);
tree.tickWhileRunning();   

rclcpp::shutdown();

if(sub_thread.joinable()){

sub_thread.join();

}

    return 0;
}

