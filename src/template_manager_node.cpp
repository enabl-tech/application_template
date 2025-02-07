/**
 * @file template_manager_node.cpp
 * @author Elias Schildge (elias.schildge@enabl-tech.de)
 * @brief 
 * @version 0.1
 * @date 2025-02-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "template_manager_node.hpp"

TemplateManagerNode::TemplateManagerNode()
        : rclcpp_lifecycle::LifecycleNode("templateManagerNodeFl"),
          templateManager(std::make_unique<TemplateManager>())
{}

CallbackReturn TemplateManagerNode::on_configure(const rclcpp_lifecycle::State &)
{
    // Create publishers
    m_thisHeartbeatPublisher = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "forklift/network_manager/heartbeat", 10);

    // Initialize and activate subscribers in active state
    m_heartbeatSubscriber = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "forklift/heartbeat/heartbeat", 10,
        std::bind(&TemplateManagerNode::ForkliftHeartbeatCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Network Manager Node configured");
    return CallbackReturn::SUCCESS;
}

CallbackReturn TemplateManagerNode::on_activate(const rclcpp_lifecycle::State & state)
{
    // Activate lifecycle entities
    LifecycleNode::on_activate(state);

    // Create timers
    m_timerMain = this->create_wall_timer(100ms,
                                            std::bind(&TemplateManagerNode::MainFunctionTimer,
                                                    this));
    m_timerHeartbeat = this->create_wall_timer(100ms,
                                                std::bind(&TemplateManagerNode::ThisHeartbeatTimer,
                                                        this));

    RCLCPP_INFO(this->get_logger(), "Network Manager Node activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn TemplateManagerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    // Reset timers
    m_timerMain.reset();
    m_timerHeartbeat.reset();
    
    // Deactivate lifecycle entities
    LifecycleNode::on_deactivate(state);

    RCLCPP_INFO(this->get_logger(), "Network Manager Node deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn TemplateManagerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    resetROSInterfaces();

    // Reset the network manager values
    templateManager = std::make_unique<TemplateManager>();

    RCLCPP_INFO(this->get_logger(), "Network Node cleaned up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn TemplateManagerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
    // Release all pointers
    resetROSInterfaces();
    templateManager.reset();

    RCLCPP_INFO(this->get_logger(), "Network Node shutdown from state %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

void TemplateManagerNode::MainFunctionTimer() { templateManager->MainFunction(); }

void TemplateManagerNode::ThisHeartbeatTimer()
{
    templateManager->HeartbeatCounterUp();
    SendHeartbeat();
}

void TemplateManagerNode::SendHeartbeat()
{
    auto nodeHeartbeatMsg = std_msgs::msg::Int32MultiArray();
    std::vector<int> vec_info = templateManager->GetterHeartbeat();
    nodeHeartbeatMsg.data = vec_info;
    m_thisHeartbeatPublisher->publish(nodeHeartbeatMsg);
}

void TemplateManagerNode::ForkliftHeartbeatCallback(const std_msgs::msg::Int32MultiArray::SharedPtr t_msg)
{
    templateManager->ForkliftHeartbeatHandler(t_msg);
}

void TemplateManagerNode::resetROSInterfaces()
{   
    // Reset timers
    m_timerMain.reset();
    m_timerHeartbeat.reset();

    // Reset publishers
    m_thisHeartbeatPublisher.reset();

    // Reset subscribers
    m_heartbeatSubscriber.reset();
}

namespace {
// Shutdown the node when an interrupt signal is received for faster systemctl restart
void signalHandler(int signum) {
    rclcpp::shutdown();
    std::exit(signum);
}
}  // namespace

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    // Set up signal handling
    std::signal(SIGINT, signalHandler);
    
    // Create and spin node
    auto node = std::make_shared<TemplateManagerNode>();
    rclcpp::spin(node->get_node_base_interface());
    
    rclcpp::shutdown();
    return 0;
}