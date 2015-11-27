/* Teleoperation System for Mobile Manipulators Framework
 *
 * Copyright (C) 2015 
 * RECONFIGURABLE CONTROL OF ROBOTIC SYSTEMS OVER NETWORKS Project
 * Robotics and Control Systems Laboratory
 * Department of Electrical Engineering
 * Sapientia Hungarian University of Transylvania
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For more details see the project homepage:
 * <http://www.ms.sapientia.ro/~martonl/MartonL_Research_TE.htm>
 */


#include "Slave_Controller_YouBot.h"
#include <iostream>
#include <cstring>

using namespace std;

Slave_Controller_YouBot::Slave_Controller_YouBot(std::string logPath,
												 double timerPeriod,
                                                 double max_force,
                                                 double max_velocity,
                                                 bool   passivityControllerEnabled,
                                                 unsigned int artificialDelayM2S,
                                                 double joint1Min,
                                                 double joint1Max,
                                                 double hapticJoint1Min,
                                                 double hapticJoint1Max,
                                                 double integralGainVelocityControl)
    : Slave_Controller(timerPeriod),
      m_maxVelocity(max_velocity),
      m_passivityControllerEnabled(passivityControllerEnabled),
      m_artificialDelayM2S(artificialDelayM2S)

{
    Slave_PO_PC_Type popcType;
    if (!passivityControllerEnabled)
    {
       popcType = No_PO_PC;
    }
    else
    {
       popcType = Delta_PO_PC;
    }
    
    m_PO_PC_J1.reset(new PO_PC_Slave_YoubotArm(logPath, "PO_PC_S_Joint1", popcType, timerPeriod, max_force, m_maxVelocity));

	// Artificial delay initialization
    if (artificialDelayM2S != 0)
	{
		Communication::ControlData empty_ControlData;
        memset(&empty_ControlData, 0, sizeof(empty_ControlData));

		for (int i = 1; i <= artificialDelayM2S; i++)
		{
			m_ArtificialDelayQueue.push(empty_ControlData);
		}
	}
}

Slave_Controller_YouBot::~Slave_Controller_YouBot()
{
}

void Slave_Controller_YouBot::SetCurrentRobotControlData(Communication::ControlData robotData)
{
    Slave_Controller::SetCurrentRobotControlData(robotData);
}

void Slave_Controller_YouBot::SetCurrentMasterControlData(Communication::ControlData masterData)
{
	if (m_artificialDelayM2S == 0)
	{
        Slave_Controller::SetCurrentMasterControlData(masterData);
	}
	else
	{
        // Implementation of artificial delay
		m_ArtificialDelayQueue.push(masterData);
		Communication::ControlData lastQueueElement =  m_ArtificialDelayQueue.front();
		m_ArtificialDelayQueue.pop();
        Slave_Controller::SetCurrentMasterControlData(lastQueueElement);
	}
}

Communication::ControlData Slave_Controller_YouBot::GetNewRobotControlData()
{
    m_newRobotControlData = m_currentMasterControlData;

    if (m_passivityControllerEnabled)
	{
        // Reads the data for the implementation of PO-PC for Joint 1
        double p_robot_J1 = m_currentRobotControlData.position[3];
        double v_robot_J1 = m_currentRobotControlData.velocity[3];
        double f_robot_J1 = m_currentRobotControlData.force[3];

        double p_master_J1 = m_currentMasterControlData.position[3];
        double v_master_J1 = m_currentMasterControlData.velocity[3];
        double f_master_J1 = m_currentMasterControlData.force[3];
        double E_remote_del_J1 = m_currentMasterControlData.energy[3];

        // Computes the controlled velocity signal, sets the new energy value to be sent
        v_robot_J1 = m_PO_PC_J1->GetControlSignal(f_robot_J1, v_robot_J1, f_master_J1, v_master_J1, E_remote_del_J1, m_newRobotControlData.energy[3], p_robot_J1, p_master_J1);

        if (v_robot_J1 < - m_maxVelocity)
        {
            v_robot_J1 = - m_maxVelocity;
        }
        else if (v_robot_J1 > m_maxVelocity)
        {
            v_robot_J1 = m_maxVelocity;
        }

        m_newRobotControlData.velocity[3] = v_robot_J1;

	}
    return Slave_Controller::GetNewRobotControlData();
}
