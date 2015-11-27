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


#ifndef MASTER_CONTROLLER_YOUBOT_H
#define MASTER_CONTROLLER_YOUBOT_H

#include "Master_Controller.h"
#include "PO_PC_Master_YoubotArm.h"
#include <queue>
#include <string>
#include <boost/shared_ptr.hpp>

/** Implements a KUKA youBot specific master side time domain passivity controller */
class Master_Controller_Youbot:
	public Master_Controller
{
	public:
		/**
		 * Constructor
		 * @param[in] logPath The path of the log files.
		 * @param[in] timerPeriod  The sampling period of the control algorithm.
		 * @param[in] max_force Maximum allowed force for the haptic device.
		 * @param[in] max_velocity Maximum velocity value of the robot.
		 * @param[in] passivityControllerEnabled If it is true the received force may be modified by the passivity controller. Otherwise the force value is returned as it is received from the slave side.
		 * @param[in] artificialDelayS2M Communication delay introduced by the application in "timerPeriod" units; Used to test the passivity controllers.
         * @param[in] joint1Min Minimum value of KUKA Youbot's joint 1 (base joint) position. Used to convert the master side position to the slave side position.
		 * @param[in] joint1Max Maximum value of KUKA Youbot's joint 1 (base joint) position. Used to convert the master side position to the slave side position.
		 * @param[in] hapticJoint1Min Minimum value of the haptic device's joint 1. Used to convert the master side position to the slave side position.
		 * @param[in] hapticJoint1Max Minimum value of the haptic device's joint 1. Used to convert the master side position to the slave side position.
		 * @param[in] integralGainVelocityControl The control gain of the position error. Used to convert the slave side position into force.
		 */
		Master_Controller_Youbot(std::string logPath,
			double timerPeriod, double max_force, double max_velocity, bool passivityControllerEnabled, unsigned int artificialDelayS2M,
			double joint1Min, double joint1Max, double hapticJoint1Min, double hapticJoint1Max, double integralGainVelocityControl);
		
		/**
		 * Destructor.
		 */
		~Master_Controller_Youbot();

		/**
		 * Passes the master side haptic device's positions, velocities, forces to the master side controller.
		 * @param[in] hapticData ControlData type structure that contains all the signals.
		 */
		void SetCurrentHapticControlData(Communication::ControlData hapticData);
		
		/**
		 * Passes the slave side robot's positions, velocities, forces and energies to the master side controller.
		 * @param[in] slaveData ControlData type structure that contains all the signals.
		 */
		void SetCurrentSlaveControlData(Communication::ControlData slaveData);
		
		/**
		 * Calculates the modified force signal using the Passivity Controller.
		 * return ControlData type structure that contains the modified force signal.
		 */
		Communication::ControlData GetNewHapticControlData();

	private:
		/** Passivity observer - controller object for haptic joint */
		boost::shared_ptr<PO_PC_Master_YoubotArm> m_PO_PC_Haptic_J1;

		/** Flag that shows the active/disable state of the passivity controller */
		bool m_passivityControllerEnabled;
		
		/** Maximum allowed force of the haptic device */
		double m_maxForce;
		
		/** Queue for implementing the artificial delay. It contains the delayed Control Data packets. Its length is given by m_artificialDelayS2M */
		std::queue<Communication::ControlData> m_ArtificialDelayQueue;
		
		/** Communication delay introduced by the application in "timerPeriod" units; Used to test the passivity controllers. */
		unsigned int m_artificialDelayS2M;
};
#endif // MASTER_CONTROLLER_YOUBOT_H
