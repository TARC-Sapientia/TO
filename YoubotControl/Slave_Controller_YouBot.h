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


#ifndef SLAVE_CONTROLLER_YOUBOT_H
#define SLAVE_CONTROLLER_YOUBOT_H

#include "Slave_Controller.h"
#include "PO_PC_Slave_YoubotArm.h"
#include <queue>
#include <boost/shared_ptr.hpp>

/** Implements a KUKA youBot specific slave side time domain passivity controller */
class Slave_Controller_YouBot :
	public Slave_Controller
{
	public:
		/**
		 * Constructor
		 * @param[in] logPath The path of the log files.
		 * @param[in] timerPeriod  The sampling period of the control algorithm.
		 * @param[in] max_force Maximum allowed force for the haptic device.
		 * @param[in] max_velocity Maximum velocity value of the robot.
		 * @param[in] passivityControllerEnabled If it is true the received velocity may be modified by the passivity controller. Otherwise the force value is returned as it is received from the slave side.
         * @param[in] artificialDelayM2S Communication delay introduced by the application in "timerPeriod" units; Used to test the passivity controllers.
         * @param[in] joint1Min Minimum value of KUKA Youbot's joint 1 (base joint) position. Used to convert the master side position to the slave side position.
         * @param[in] joint1Max Maximum value of KUKA Youbot's joint 1 (base joint) position. Used to convert the master side position to the slave side position.
         * @param[in] hapticJoint1Min Minimum value of the haptic device's joint 1. Used to convert the master side position to the slave side position.
         * @param[in] hapticJoint1Max Minimum value of the haptic device's joint 1. Used to convert the master side position to the slave side position.
         * @param[in] integralGainVelocityControl The control gain of the position error. Used to convert the slave side position into force.
		 */
        Slave_Controller_YouBot(std::string logPath,
								double timerPeriod, double max_force, double max_velocity, bool passivityControllerEnabled, unsigned int artificialDelayM2S,
                                double joint1Min, double joint1Max, double hapticJoint1Min, double hapticJoint1Max, double integralGainVelocityControl);
		
		/**
		 * Destructor.
		 */
        ~Slave_Controller_YouBot();
		
		/**
		 * Passes the slave side robot's positions, velocities, forces to the slave side controller.
		 * @param[in] robotData ControlData type structure that contains all the signals.
		 */
        void SetCurrentRobotControlData(Communication::ControlData robotData);
		
		/**
		 * Passes the master side haptic device's positions, velocities, forces and energies to the slave side controller.
		 * @param[in] masterData ControlData type structure that contains all the signals.
		 */
        void SetCurrentMasterControlData(Communication::ControlData masterData);
		
		/**
		 * Calculates the modified velocity signal using the Passivity Controller.
		 * return ControlData type structure that contains the modified velocity signal.
		 */
        Communication::ControlData GetNewRobotControlData();
	private:
		/** Passivity controller object for the youBot arm joint */
        boost::shared_ptr<PO_PC_Slave_YoubotArm> m_PO_PC_J1;

		/** Flag that shows the active/disable state of the passivity controller */
		bool m_passivityControllerEnabled;

        /** Maximum allowed velocity of the haptic device */
        double m_maxVelocity;

		/** Queue for implementing the artificial delay. It contains the delayed Control Data packets. Its length is given by m_artificialDelayS2M */
        std::queue<Communication::ControlData> m_ArtificialDelayQueue;
		
		/** Communication delay introduced by the application in "timerPeriod" units; Used to test the passivity controllers. */
		unsigned int m_artificialDelayM2S;
};

#endif // SLAVE_CONTROLLER_YOUBOT_H
