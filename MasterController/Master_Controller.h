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


#ifndef MASTER_CONTROLLER_H
#define MASTER_CONTROLLER_H

#include "CommunicationData.h"

/** Stores the possible time domain passivity controller types on the master (haptic device) side*/
enum Master_PO_PC_Type
{
    /** Passivity controller is not active, the force signal is returned as it was received */
    No_PO_PC,

    /** The passivity controller is active */
    Original_PO_PC,

    /** Implements the filtered, bounded output passivity controller */
    Delta_PO_PC,
};

/** Base class for master side time domain passivity controller */
class Master_Controller
{
public:
	/**
     * Constructor
	 * @param[in] timerPeriod  The sampling period of the control algorithm.
     */
	Master_Controller(double timerPeriod);
	
	/**
     * Destructor.
     */
	~Master_Controller();
	
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

protected:

	/** Stores the sampling period used to implement the control */
	double m_T;
	
	/** Stores the received data (position, velocity, force, energy) from the master side haptic device */
	Communication::ControlData m_currentHapticControlData;
	
	/** Stores the computed data by the controller (position, velocity, force, energy) for the master side haptic device */
	Communication::ControlData m_newHapticControlData;
	
	/** Stores the received data (position, velocity, force, energy) from the slave side robot */
	Communication::ControlData m_currentSlaveControlData;
};

#endif // MASTER_CONTROLLER_H 
