/**
 * @file template_manager_node.hpp
 * @author Elias Schildge (elias.schildge@enabl-tech.de)
 * @brief 
 * @version 0.1
 * @date 2025-02-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "template_manager.hpp"

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TemplateManagerNode : public rclcpp_lifecycle::LifecycleNode
{
   public:
    TemplateManagerNode();

   protected:
    // Lifecycle callbacks
    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

   private:
    /**
     * @brief Main function timer, executes the main function of the manager class as callback
     */
    void MainFunctionTimer();

    /**
     * @brief Heartbeat timer, executes the heartbeat function of the manager class as callback and sends the heartbeat message
     */
    void ThisHeartbeatTimer();

    /**
     * @brief Sends the heartbeat message to the forklift heartbeat manager
     */
    void SendHeartbeat();

    /**
     * @brief Callback function for the incoming forklift heartbeat message
     * 
     * @param t_msg The incoming forklift heartbeat message
     */
    void ForkliftHeartbeatCallback(const std_msgs::msg::Int32MultiArray::SharedPtr t_msg);

    /**
     * @brief Resets the ROS interfaces
     */
    void resetROSInterfaces();

    // Application manager object
    std::unique_ptr<TemplateManager> templateManager;

    // Timers
    rclcpp::TimerBase::SharedPtr m_timerMain;
    rclcpp::TimerBase::SharedPtr m_timerHeartbeat;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32MultiArray>::SharedPtr m_thisHeartbeatPublisher;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr m_heartbeatSubscriber;
};