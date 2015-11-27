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


#ifndef HAPTICMANAGERNOVINTFALCON_H 
#define HAPTICMANAGERNOVINTFALCON_H 

#include <vector>
#include <time.h>
#include <boost/shared_ptr.hpp>
#include "IHapticXmlParser.h"
#include "Log.h"

//The units of measure for the HDAL interface are: 
//	Distance meters 
//	Force newtons
//	Time seconds
//
//The coordinate system used by HDAL is a right hand coordinate system: 
//	X increases to the right 
//	Y increases upward
//	Z increases toward the user. 
//	
//The origin (X = 0, Y = 0, Z = 0) is approximately at the center of the device workspace.

namespace Haptics
{
    /** Enum for possible haptic [NOVINT] devices. */
	enum HapticDevice
	{
		NOVINT_H1 = 2, // 0,1 is reserved for PhantomOmni
		NOVINT_H2 = 3 
	};

    /** Enum for possible buttons. */
	enum Button
	{
        ButtonRear,
		ButtonLeft,
		ButtonFront,
		ButtonRight
	};

    /**
     * The Novint Falcon Haptic device management interface which
     * encapsulates the low level haptic control methods.
     * Contains the necessary methods to send/receive data to/from the device.
     */
	class HapticManagerNovintFalcon
	{

	public:

        /**
         * Constructor.
         * @param[in] configFileName: The name of the configuration file including path to the file.
         */
		HapticManagerNovintFalcon(std::string configFileName);

        /**
         * Destructor.
         */
        ~HapticManagerNovintFalcon();

        /**
         * Initialize all detectable devices.
         * Returns true if successful, otherwise false is returned.
         */
		bool Initialize();


        /**
         * Clean up: undoes the setup in reverse order.
         */
		void CleanUp();

        /**
         * Check if the haptic device is connected.
         * @param device The selected device.
         * @return True if the device is connected, otherwise false is returned.
         */
		bool IsConnected(HapticDevice device);

        /**
         * Check if the device is calibrated.
         * @param device The selected device.
         * @return True if the device is calibrated, otherwise false is returned.
         */
		bool IsCalibrated(HapticDevice device);

        /**
         * Get the position of the device.
         * @param device The selected device.
         * @return The position [m] of the device.
         */
		std::vector<double> GetPosition(HapticDevice device);

        /**
         * Get the velocity of the device.
         * @param device The selected device.
         * @return The velocity [m/s] of the device.
         */
		std::vector<double> GetVelocity(HapticDevice device);

        /**
         * Get the extents of the device workspace.
         * Right-handed coordinates:
         *  left-right is the x-axis, right is greater than left
         *  bottom-top is the y-axis, top is greater than bottom
         *  near-far is the z-axis, near is greater than far
         * The workspace center is (0,0,0).
         * @param device The selected device.
         * @return The dimensions [m] of the workspace as an array:
         *          minx, miny, minz, maxx, maxy, maxz, left, bottom, far, right, top, near
         */
		std::vector<double> GetWorkspaceDimensions(HapticDevice device);

        /**
         * Sets the given force value to the selected haptic device.
         * @param device The selected device.
         * @param force The selected force [N] value to be sent.
         */
		void SetForce(HapticDevice device, std::vector<double> force);

        /**
         * Gets the given force value to the selected haptic device.
         * @param device The selected device.
         * @return force The given force [N] value.
         */
		std::vector<double> GetForce(HapticDevice device);

        /**
         * Queries the state of the light button on the selected haptic device.
         * @param device The selected device.
         * @param button The selected button.
         * @return True if the button is pressed, otherwise false is returned.
         */
		bool IsButtonPressed(HapticDevice device, Button button);

        /**
         * Retreives the configuration of the haptic manager.
         * @return A pointer to the configuration struct is returned.
         */
		const boost::shared_ptr<Utils::HapticManagerConfig> GetConfig();

	private:
		
        // Variables used to calculate the velocity

        /** The current time instant. */
        clock_t m_T_k;

        /** The previous time instant. */
        clock_t m_T_k_1;

        /** The previous position. */
		std::vector<double> m_Position_k_1;

        /** The previous velocity. */
		std::vector<double> m_Velocity_k_1;

        /** The configuration struct. */
		boost::shared_ptr<Utils::HapticManagerConfig> m_Config;

        /** The log file for the haptic device. */
		boost::shared_ptr<Utils::Log> m_LogH1;
	};
}
#endif // HAPTICMANAGERNOVINTFALCON_H
