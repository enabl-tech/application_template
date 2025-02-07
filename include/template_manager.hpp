/**
 * @file template_manager.hpp
 * @author Elias Schildge (elias.schildge@enabl-tech.de)
 * @brief 
 * @version 0.1
 * @date 2025-02-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef template_manager_H_
#define template_manager_H_

#include <unistd.h>
#include <iostream>

#include "std_msgs/msg/int32_multi_array.hpp"

class TemplateManager
{
   public:
    /**
     * @brief Main function
     */
    void MainFunction();

    /**
     * @brief Increments the heartbeat counter
     */
    void HeartbeatCounterUp();

    /**
     * @brief Handles the forklift heartbeat message, resets the heartbeat counter and updates the heartbeat state
     * 
     * @param t_msg The incoming forklift heartbeat message
     */
    void ForkliftHeartbeatHandler(const std_msgs::msg::Int32MultiArray::SharedPtr t_msg)
    {
        m_forkliftHeartbeatCounter = 0;
        m_forkliftHeartbeat = static_cast<TemplateManager::HeartbeatState>(t_msg->data[1]);
    }

    /**
     * @brief Returns the heartbeat vector
     * 
     * @return std::vector<int> {counter, heartbeat}
     */
    std::vector<int> GetterHeartbeat()
    {
        std::vector<int> vectorInfo{(int)m_thisCounter, (int)m_thisHeartbeat};
        return vectorInfo;
    }

   private:
    /**
	 * @brief Handles the heartbeat
     * @details Checks if the forklift heartbeat is timed out or not, or if the application is in error state. 
     *          Updates the application heartbeat state accordingly.
	 */
    void HandleHeartbeat();

    /**
	 * @brief Checks wether the forklift heartbeat manager message is timed out
	 * 
	 */
    void CheckForkliftHeartbeatTimeOut();

    enum HeartbeatState
    {
        sWaiting,
        sNormal,
        sError,
    };

    HeartbeatState m_forkliftHeartbeat = sWaiting;
    HeartbeatState m_thisHeartbeat = sWaiting;

    const uint8_t m_thisMaxCounter = 100;
    uint8_t m_thisCounter = 0;

    int m_forkliftHeartbeatMaxTime = 10;
    int m_forkliftHeartbeatCounter = 0;
    bool m_forkliftHeartbeatTimedOut = false;

    bool m_thisError = false;
};

#endif