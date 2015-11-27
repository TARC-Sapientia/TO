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


#ifndef PO_PC_SLAVE_YOUBOTARM_H
#define PO_PC_SLAVE_YOUBOTARM_H

#include "Slave_Controller.h"
#include <boost/shared_ptr.hpp>
#include <string>
#include <Log.h>

using namespace std;

/** Implements a time domain passivity controller (Passivity Observer - Passivity Controller PO-PC structure) on the slave (robot) side*/
class PO_PC_Slave_YoubotArm
{
	public:
		/**
		 * Constructor
		 * @param[in] logPath The path of the log file.
		 * @param[in] logName The name of the log file.
		 * @param[in] po_pc_Type Structure that stores the possible time domain passivity controller types. 
		 * @param[in] T Sampling period of the control algorithm.
		 * @param[in] max_force Maximum allowed force for the haptic device.
		 * @param[in] max_velocity Maximum velocity value of the robot.
		 */
        PO_PC_Slave_YoubotArm(std::string logPath, std::string logName,
              Slave_PO_PC_Type po_pc_Type,
			  double T,
			  double max_force,
			  double max_velocity);
			  
		/**
		* Destructor
		*/
        ~PO_PC_Slave_YoubotArm();

        /** 
		* Resets to zero energies, velocities, forces and other variables used by the PO-PC controller.
		*/
        void Reset_PO_PC();

		/** 
		* Interface for the time domain passivity controller. Computes the modified force signal.
		* @param[in] f_local The force of the robot device (slave side).
		* @param[in] v_local The velocity of the robot (slave side).
		* @param[in] f_remote_del The force of the robot received from the master side.
		* @param[in] v_remote_del The velocity of the robot received from the master side.
        * @param[out] E_remote_del The energy received from the master side.
        * @param[in] E_local_toSend The energy computed on the slave side. It will be transmitted to the master side.
		* @param[in] p_local The position of the robot (slave side).
		* @param[in] p_remote_del The position of the haptic device received from the master side.
		* @return The modified velocity value for the robot.
		*/
        double GetControlSignal(double f_local, double v_local, double f_remote_del, double v_remote_del, double E_remote_del, double& E_local_toSend, double p_local, double p_remote_del);

	private:
		/** 
		* Implements the Passivity Observer - Computes the slave side IN and OUT energies.
		* @param[in] f The force of the robot (slave side).
		* @param[in] v The velocity of the haptic device received form the master side.
		* @param[in] PO_DZ_Width Dead Zone Width: The energies are incremented if the power is over the dead zone value.
		*/
		void Passivity_Observer(double f, double v, double PO_DZ_Width);

        /** 
		* Implements the Passivity Controller - Computes the modified velocity signal for the slave side robot.
		* @param[in] f_local The force of the robot (slave side).
		* @param[in] v_local The velocity of the robot (slave side).
		* @param[in] f_remote_del The force of the haptic device received from the master side.
		* @param[in] v_remote_del The velocity of the haptic device received from the master side.
		* @param[in] E_IN_del The energy received from the master side.
		* @return The modified velocity value for the robot.
		*/
		double Passivity_Controller(double f_local, double v_local, double f_remote_del, double v_remote_del, double E_IN_del);

        /**
		* Implements a dead zone function
		* @param[in] value Input of the dead zone function.
		* @param[in] DZ Dead zone width.
		* @return Output of the dead zone function
		*/
        double DeadZone(double value, double DZ);

		/**
		* Implements a sign function
		* @param[in] value Input of the sign function.
		* @return Output of the sign function
		*/
		double Sign(double value);

		/** Structure that stores the possible time domain passivity controller types. */
        Slave_PO_PC_Type m_PO_PC_Type;
		
		/** Sampling period of the control algorithm. */
		double m_T;
		
		/** Maximum allowed force for the haptic device. */
		double m_max_force;
		
		/** Maximum velocity value of the robot. */
		double m_max_velocity;

		/** Integral time constant of the filtered, bounded output passivity controller */
		double m_TI_S;
		
		/** Minimum value of the denominator of the filtered, bounded output passivity controller */
		double m_delta_S;
		
		/** Dynamic internal state of the filtered, bounded output passivity controller */
		double m_z;
		
		/** Slave side input energy */
		double m_E_local_IN;
		
		/** Slave side output energy */
		double m_E_local_OUT;
		
		/** Energy dissipated by the passivity controller */
		double m_PC_Energy;
		
		/** Control signal computed by the passivity controller which modifies additively the received velocity value */
		double m_PC_signal;
		
		/** Controlled velocity signal: received velocity and m_PC_signal */
        double m_ControlSignal;

		/** Filtered control signal computed by the passivity controller */
		double m_PC_signal_F;
		
		/** Filter coefficient for the control signal filter */
		double m_alpha_PC_signal_F;

		/** Control signal value in the previous control sample */
		double m_PC_signal_old;

        /** Received energy from the master side in the previous control sample */
        double m_E_Remote_del_old;
		
		/** The delayed velocity of the haptic device (from the master side) in the previous control sample */
		double m_v_old;
		
		/** Force of the robot in the previous sample */
		double m_f_old;

		/** Power threshold value for energy computation in the passivity observer */
		double m_P_S_treshold;
		
		/** Force threshold value for control signal computation in the non-modified passivity controller */
		double m_f_S_treshold;

		/** Output file stream for logging the variables used by the time domain passivity controller */
        boost::shared_ptr<Utils::Log> m_PO_PC_Logger;
};

#endif // PO_PC_SLAVE_YOUBOTARM_H
