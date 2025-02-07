/**
 * @file template_manager.cpp
 * @author Elias Schildge (elias.schildge@enabl-tech.de)
 * @brief 
 * @version 0.1
 * @date 2025-02-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "template_manager.hpp"

void TemplateManager::MainFunction()
{
    HandleHeartbeat();

    /**
     * 
     * Add your main function here
     * 
     *  
     */ 
}

void TemplateManager::HandleHeartbeat()
{
    CheckForkliftHeartbeatTimeOut();

    if (m_thisError) {
        m_thisHeartbeat = sError;
    } else if (m_forkliftHeartbeatTimedOut || m_forkliftHeartbeat == sWaiting || m_forkliftHeartbeat == sError) {
        if (m_thisHeartbeat != sError) {
            m_thisHeartbeat = sWaiting;
        }
    } else {
        m_thisHeartbeat = sNormal;
    }
}

void TemplateManager::CheckForkliftHeartbeatTimeOut()
{
    m_forkliftHeartbeatCounter++;

    if (m_forkliftHeartbeatCounter >= m_forkliftHeartbeatMaxTime) {
        m_forkliftHeartbeatCounter = m_forkliftHeartbeatMaxTime;
        m_forkliftHeartbeatTimedOut = true;
    } else {
        m_forkliftHeartbeatTimedOut = false;
    }
}

void TemplateManager::HeartbeatCounterUp()
{
    m_thisCounter++;

    if (m_thisCounter > m_thisMaxCounter) { m_thisCounter = 0; }
}