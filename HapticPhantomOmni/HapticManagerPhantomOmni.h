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


#ifndef HAPTICMANAGER_H
#define HAPTICMANAGER_H

#include <boost/shared_ptr.hpp>
#include "IHapticXmlParser.h"
#include "Log.h"

namespace Haptics
{	
    /** Enum for possible haptic [PHANToM] devices. */
    enum HapticDevice
    {
        PHANToM_H1 = 0,
        PHANToM_H2 = 1
    };

    /** Enum for possible button states. */
    enum ButtonState
    {
        None = 0,
        DarkPressed = 1,
        LightPressed = 2,
        BothPressed = 3
    };

    /**
     * Structure holding the haptic device data.
     * The information is based on the current state of the haptic device pointer.
     */
    struct HapticData
    {
        /** Array holding the position [mm] values. */
        double position[3];

        /** Array holding the velocity [mm/s] values. */
        double velocity[3];

        /** Array holding the joint angle [rad] values. */
        double jointAngles[3];

        /** Array holding the gimbal angle [rad] values. */
        double gimbalAngles[3];

        /** Array holding the force [N] values. */
        double force[3];

        /** Value indicating the button state. */
        ButtonState buttonState;
    };

    /**
     * The PHANToM Omni Haptic device management interface which
     * encapsulates the low level haptic control methods.
     * Contains the necessary methods to send/receive data to/from the device.     
     */
    class HapticManagerPhantomOmni
	{
	public:

        /**
         * Constructor.
         * Starts the scheduler.
         * @param[in] configFileName: The name of the configuration file including path to the file.
         */
        HapticManagerPhantomOmni(std::string configFileName);

        /**
         * Destructor.
         * Stops the scheduler.
        */
        ~HapticManagerPhantomOmni();

        /**
         * Get the state of the device.
         * @param device The selected device.
         * @return The device data in a HapticData struct.
         */
		HapticData GetData(HapticDevice device);

        /**
         * Sets the given force value to the selected haptic device.
         * @param device The selected device.
         * @param force The selected force [N] value to be sent.
         */
		void SetForce(HapticDevice device, double force[3]);

        /**
         * Check if the haptic device is connected.
         * @param device The selected device.
         * @return True if the device is connected, otherwise false is returned.
         */
		bool IsConnected(HapticDevice device);

        /**
         * Check the state of the dark button on the selected haptic device.
         * @param device The selected device.
         * @return True if the dark button is pressed, otherwise false is returned.
         */
		bool IsDarkGreyButtonPressed(HapticDevice device);

        /**
         * Check the state of the light button on the selected haptic device.
         * @param device The selected device.
         * @return True if the light button is pressed, otherwise false is returned.
         */
		bool IsLightGreyButtonPressed(HapticDevice device);
		
        /**
         * Retrieves the configuration of the haptic manager.
         * @return A pointer to the configuration struct is returned.
         */
		const boost::shared_ptr<Utils::HapticManagerConfig> GetConfig();

	private:		

        /** The configuration struct. */
		boost::shared_ptr<Utils::HapticManagerConfig> m_Config;

        /** The log file for the first haptic device. */
		boost::shared_ptr<Utils::Log> m_LogH1;

        /** The log file for the second haptic device. */
        boost::shared_ptr<Utils::Log> m_LogH2;
	};
} // end namespace
#endif // HAPTICMANAGER_H
