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


#include "Master_Controller_Youbot.h"
#include <string.h>

Master_Controller_Youbot::Master_Controller_Youbot(std::string logPath,
													double timerPeriod, double max_force, double max_velocity, bool passivityControllerEnabled, unsigned int artificialDelayS2M,
													double joint1Min, double joint1Max, double hapticJoint1Min, double hapticJoint1Max, double integralGainVelocityControl)
	:	
		Master_Controller(timerPeriod),
		m_maxForce(max_force),
		m_passivityControllerEnabled(passivityControllerEnabled),
		m_artificialDelayS2M(artificialDelayS2M)
{
	Master_PO_PC_Type popcType;
	if (!passivityControllerEnabled)
	{
		popcType = No_PO_PC;
	}
	else
	{
		popcType = Delta_PO_PC;
	}

	// PO-PC for haptic that deals with youBot arm
	m_PO_PC_Haptic_J1.reset(new PO_PC_Master_YoubotArm(logPath, "PO_PC_M_LOG_Master_J1", popcType, timerPeriod, max_force, max_velocity));
	
	// Artificial delay initialization	
	if (artificialDelayS2M != 0)
	{
	   Communication::ControlData empty_ControlData;
	   memset(&empty_ControlData, 0, sizeof(empty_ControlData));
	   for (unsigned int i = 1; i <= artificialDelayS2M; i++)
	   {
			m_ArtificialDelayQueue.push(empty_ControlData);
	   }
	}
}

Master_Controller_Youbot::~Master_Controller_Youbot()
{
}

void Master_Controller_Youbot::SetCurrentHapticControlData(Communication::ControlData hapticData)
{
	 Master_Controller::SetCurrentHapticControlData(hapticData);
}

void Master_Controller_Youbot::SetCurrentSlaveControlData(Communication::ControlData slaveData)
{
	if (m_artificialDelayS2M == 0)
	{
		Master_Controller::SetCurrentSlaveControlData(slaveData);
	}
	else
	{
		// Implementation of artificial delay
		m_ArtificialDelayQueue.push(slaveData);
		Communication::ControlData lastQueueElement =  m_ArtificialDelayQueue.front();
		m_ArtificialDelayQueue.pop();
		Master_Controller::SetCurrentSlaveControlData(lastQueueElement);
	}
}

Communication::ControlData Master_Controller_Youbot::GetNewHapticControlData()
{
	m_newHapticControlData = m_currentSlaveControlData;
	
	if( m_passivityControllerEnabled)
	{
		// PO-PC control for Haptic Joint 1
		double p_haptic_x = m_currentHapticControlData.position[3];
		double v_haptic_x = m_currentHapticControlData.velocity[3];
		double f_haptic_x = m_currentHapticControlData.force[3];

		double p_slave_x = m_currentSlaveControlData.position[3];
		double v_slave_x = m_currentSlaveControlData.velocity[3];
		double f_slave_x = m_currentSlaveControlData.force[3];
		double E_remote_del_x = m_currentSlaveControlData.energy[3];

		// Computes the controlled force signal, sets the new energy value to be sent to the slave side
		double f_haptic_x_controlled = m_PO_PC_Haptic_J1->GetControlSignal(f_haptic_x, v_haptic_x, f_slave_x, v_slave_x, E_remote_del_x, m_newHapticControlData.energy[3], p_haptic_x, p_slave_x); 

		if (f_haptic_x_controlled < - m_maxForce)
		{
			f_haptic_x_controlled = - m_maxForce;
		}
		else if (f_haptic_x_controlled > m_maxForce)
		{
			f_haptic_x_controlled = m_maxForce;
		}

		m_newHapticControlData.force[3] = f_haptic_x_controlled;
	}
	
	return Master_Controller::GetNewHapticControlData();
}
