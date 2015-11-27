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


#include "HapticManagerPhantomOmni.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <boost/thread.hpp>
#include <HD/hd.h>
#include <HL/hl.h>
#include "Utilities.h"

using namespace std;
using namespace Utils;

namespace Haptics
{
    // Device handles.
	HHD hHD_H1;
	HHD hHD_H2;

    // The scheduler handle.
	HDSchedulerHandle hDSchedulerHandle;

    // The current haptic device state stored in a HapticData struct.
	HapticData deviceState_H1;
	HapticData deviceState_H2;

    // Values indicating the availability of the haptic devices.
	bool isDeviceConnected_H1;
	bool isDeviceConnected_H2;

    // The current force values that will sent in the next loop.
	HDdouble forceToHaptic_H1[3];
	HDdouble forceToHaptic_H2[3];
	
    // Synchronization mutex.
	boost::recursive_mutex mutex;

    // Debug function
	void PrintState(HapticData deviceState)
	{
		static int counter = 0;
		counter++;
		if (counter > 50) 
		{
			counter = 0;
			cout << "Buttons "; 
			if (deviceState.buttonState == DarkPressed) cout << "Dark pressed." << endl;
			else if (deviceState.buttonState == LightPressed) cout << "Light pressed." << endl;
			else if (deviceState.buttonState == BothPressed) cout << "Both pressed." << endl;

			PrintVector("Position", deviceState.position, 3);
			PrintVector("Velocity", deviceState.velocity, 3);
			PrintVector("JointAng", deviceState.jointAngles,3);
			PrintVector("GimbalAn", deviceState.gimbalAngles,3);
			PrintVector("Force   ", deviceState.force, 3);
		}
	}

    // Helper function which converts the Haptic data to a corresponding string.
	string HapticDataToString(HapticData deviceState)
	{
		stringstream ss;

		for (unsigned int i = 0; i < 3; i++)
		{
			ss << "p" << i << ": " << deviceState.position[i] << "\t";
			ss << "f" << i << ": " << deviceState.force[i] << "\t";
			ss << "v" << i << ": " << deviceState.velocity[i] << "\t";
			ss << "g" << i << ": " << deviceState.gimbalAngles[i] << "\t";
			ss << "j" << i << ": " << deviceState.jointAngles[i] << "\t";
		}

		ss << "b: " << deviceState.buttonState;
		ss << endl;

		return ss.str();
	}

    // The scheduler callback function which is continuously executed
    // In each loop the connected device is queried, the data is stored in a HapticData struct,
    // and the force values are assigned to the devices.
	HDCallbackCode HDCALLBACK AsyncSchedulerCallback(void *pUserData)
	{
		HDErrorInfo error;
		int button1 = 0;
		int button2 = 0;
		
		if (isDeviceConnected_H1) hdBeginFrame(hHD_H1);
		if (isDeviceConnected_H2) hdBeginFrame(hHD_H2);

		mutex.lock();

		// Phantom 1
		if (isDeviceConnected_H1)
		{
			hdMakeCurrentDevice(hHD_H1);
			hdGetIntegerv(HD_CURRENT_BUTTONS, &button1); deviceState_H1.buttonState = static_cast<ButtonState>(button1);
			hdGetDoublev(HD_CURRENT_POSITION, deviceState_H1.position);
			hdGetDoublev(HD_CURRENT_VELOCITY, deviceState_H1.velocity);
			hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, deviceState_H1.gimbalAngles);
			hdGetDoublev(HD_CURRENT_JOINT_ANGLES, deviceState_H1.jointAngles);
			hdGetDoublev(HD_CURRENT_FORCE, deviceState_H1.force);

			hdSetDoublev(HD_CURRENT_FORCE, forceToHaptic_H1);
		}

		// Phantom 2
		if (isDeviceConnected_H2)
		{
			hdMakeCurrentDevice(hHD_H2);
			hdGetIntegerv(HD_CURRENT_BUTTONS, &button2); deviceState_H2.buttonState = static_cast<ButtonState>(button2);
			hdGetDoublev(HD_CURRENT_POSITION, deviceState_H2.position);
			hdGetDoublev(HD_CURRENT_VELOCITY, deviceState_H2.velocity);
			hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, deviceState_H2.gimbalAngles);
			hdGetDoublev(HD_CURRENT_JOINT_ANGLES, deviceState_H2.jointAngles);
			hdGetDoublev(HD_CURRENT_FORCE, deviceState_H2.force);

			hdSetDoublev(HD_CURRENT_FORCE, forceToHaptic_H2);
		}

		mutex.unlock();

		if (isDeviceConnected_H1) hdEndFrame(hHD_H1);
		if (isDeviceConnected_H2) hdEndFrame(hHD_H2);

		//  Check if an error occurred while attempting to render the force
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			cout << "Haptic Manager error: " << hdGetErrorString(error.errorCode) << endl;
			
			return HD_CALLBACK_DONE;
		}

		return HD_CALLBACK_CONTINUE;
	}

    HapticManagerPhantomOmni::HapticManagerPhantomOmni(string configFileName)
	{	
		memset(&deviceState_H1, 0, sizeof(deviceState_H1));
		memset(&deviceState_H2, 0, sizeof(deviceState_H2));
	
		memset(forceToHaptic_H1, 0, sizeof(forceToHaptic_H1));
		memset(forceToHaptic_H2, 0, sizeof(forceToHaptic_H2));

		isDeviceConnected_H1 = false;
		isDeviceConnected_H2 = false;
		
		HapticXmlParserPtr xmlParser = getHapticXmlParser();
		
		try
		{
			m_Config = xmlParser->ParseHapticManagerConfig(configFileName);
		}		
		catch (exception* e)
		{
			cout << "Haptic Manager: Failed to parse configuration file: " << configFileName << endl;
            cout << "Haptic Manager: Error: " << e->what() << endl;
		}

		if (m_Config->devices.size() == 0) 
		{
			cout << "Haptic Manager: Configuration file contains no devices information." << endl;
			return;
		}
		
		HDErrorInfo error;

		hHD_H1 = hdInitDevice(m_Config->devices[PHANToM_H1]->name.c_str());
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			cout << "Haptic Manager: Device: " << m_Config->devices[PHANToM_H1]->name << " is NOT connected." << endl;
		}
		else
		{
			isDeviceConnected_H1 = true;

			if (m_Config->devices[PHANToM_H1]->logConfig->enabled == true)
			{
				m_LogH1.reset(new Utils::Log(m_Config->devices[PHANToM_H1]->logConfig->logPath, m_Config->devices[PHANToM_H1]->logConfig->logName));
			}
		}

		if (m_Config->devices.size() > 1)
		{		
			hHD_H2 = hdInitDevice(m_Config->devices[PHANToM_H2]->name.c_str());
			if (HD_DEVICE_ERROR(error = hdGetError()))
			{ 
				cout << "Haptic Manager: Device: " << m_Config->devices[PHANToM_H2]->name << " is NOT connected." << endl;
			}
			else
			{
				isDeviceConnected_H2 = true;

				if (m_Config->devices[PHANToM_H2]->logConfig->enabled == true)
				{
					m_LogH2.reset(new Utils::Log(m_Config->devices[PHANToM_H2]->logConfig->logPath, m_Config->devices[PHANToM_H2]->logConfig->logName));
				}
			}
		}

		hDSchedulerHandle = hdScheduleAsynchronous(AsyncSchedulerCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

		if (isDeviceConnected_H1) 
		{
			hdMakeCurrentDevice(hHD_H1); 
			hdEnable(HD_FORCE_OUTPUT);
		}

		if (isDeviceConnected_H2)
		{
			hdMakeCurrentDevice(hHD_H2);
			hdEnable(HD_FORCE_OUTPUT);
		}
		hdSetSchedulerRate(1000 /* Hz */);

		// If multiple devices are used, each needs to be initialized separately.
		// However, there is only one scheduler so it just needs to be started once. 
		// hdStartScheduler() starts the scheduler and should be the last call in initialization. 
		// Asynchronous calls should be scheduled before this so that they begin executing as
		// soon as the scheduler is turned on. 
		hdStartScheduler();
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
            cout << "Haptic Manager: Failed to start Phantom Omni haptic scheduler. Restart application and check haptic device(s)." << endl;
		}
	}

    bool HapticManagerPhantomOmni::IsConnected(HapticDevice device)
	{
		if (device == PHANToM_H1)
		{
			return isDeviceConnected_H1;
		}

		if (device == PHANToM_H2)
		{
			return isDeviceConnected_H2;
		}

		return false;
	}

    HapticManagerPhantomOmni::~HapticManagerPhantomOmni()
	{
		hdStopScheduler();
		hdUnschedule(hDSchedulerHandle);
		hdDisableDevice(hHD_H1);
		hdDisableDevice(hHD_H2);
	}

    bool HapticManagerPhantomOmni::IsDarkGreyButtonPressed(HapticDevice device)
	{
		bool ret = false;
		if (device == PHANToM_H1)
		{
			mutex.lock();
			ret = ((deviceState_H1.buttonState == DarkPressed) || (deviceState_H1.buttonState == BothPressed));
			mutex.unlock();
		}
		
		if (device == PHANToM_H2)
		{
			mutex.lock();
			ret = ((deviceState_H2.buttonState == DarkPressed) || (deviceState_H2.buttonState == BothPressed));
			mutex.unlock();
		}

		return ret;
	}

    bool HapticManagerPhantomOmni::IsLightGreyButtonPressed(HapticDevice device)
	{
		bool ret = false;
		if (device == PHANToM_H1)
		{
			mutex.lock();
			ret = ((deviceState_H1.buttonState == LightPressed) || (deviceState_H1.buttonState == BothPressed));
			mutex.unlock();
		}
		
		if (device == PHANToM_H2)
		{
			mutex.lock();
			ret = ((deviceState_H2.buttonState == LightPressed) || (deviceState_H2.buttonState == BothPressed));
			mutex.unlock();
		}

		return ret;
	}
	
    void HapticManagerPhantomOmni::SetForce(HapticDevice device, double force[3])
	{
		if (isDeviceConnected_H1 == true && device == PHANToM_H1)
		{
			mutex.lock();

			forceToHaptic_H1[0] = m_Config->devices[PHANToM_H1]->constraintsConfig->forceAlphaFilter * forceToHaptic_H1[0] + (1 - m_Config->devices[PHANToM_H1]->constraintsConfig->forceAlphaFilter) * force[0];
			forceToHaptic_H1[1] = m_Config->devices[PHANToM_H1]->constraintsConfig->forceAlphaFilter * forceToHaptic_H1[1] + (1 - m_Config->devices[PHANToM_H1]->constraintsConfig->forceAlphaFilter) * force[1];
			forceToHaptic_H1[2] = m_Config->devices[PHANToM_H1]->constraintsConfig->forceAlphaFilter * forceToHaptic_H1[2] + (1 - m_Config->devices[PHANToM_H1]->constraintsConfig->forceAlphaFilter) * force[2];

			ClampFunction<double>(forceToHaptic_H1, 3, m_Config->devices[PHANToM_H1]->constraintsConfig->maxForce);

			mutex.unlock();
		}

		if (isDeviceConnected_H2 == true && device == PHANToM_H2)
		{
			mutex.lock();
					
			forceToHaptic_H2[0] = m_Config->devices[PHANToM_H2]->constraintsConfig->forceAlphaFilter * forceToHaptic_H2[0] + (1 - m_Config->devices[PHANToM_H2]->constraintsConfig->forceAlphaFilter) * force[0];
			forceToHaptic_H2[1] = m_Config->devices[PHANToM_H2]->constraintsConfig->forceAlphaFilter * forceToHaptic_H2[1] + (1 - m_Config->devices[PHANToM_H2]->constraintsConfig->forceAlphaFilter) * force[1];
			forceToHaptic_H2[2] = m_Config->devices[PHANToM_H2]->constraintsConfig->forceAlphaFilter * forceToHaptic_H2[2] + (1 - m_Config->devices[PHANToM_H2]->constraintsConfig->forceAlphaFilter) * force[2];

			ClampFunction<double>(forceToHaptic_H2, 3, m_Config->devices[PHANToM_H2]->constraintsConfig->maxForce);

			mutex.unlock();
		}
	}

    HapticData HapticManagerPhantomOmni::GetData(HapticDevice device)
	{
		HapticData deviceState;
		memset(&deviceState, 0, sizeof(deviceState));

		if (device == PHANToM_H1 && isDeviceConnected_H1 == true)
		{
			mutex.lock();
			deviceState = deviceState_H1;
			mutex.unlock();

			if (m_Config->devices[PHANToM_H1]->logConfig->enabled == true)
			{
				m_LogH1->Write(HapticDataToString(deviceState));
			}
		}

		if (device == PHANToM_H2 && isDeviceConnected_H2 == true)
		{
			mutex.lock();
			deviceState = deviceState_H2;
			mutex.unlock();

			if (m_Config->devices[PHANToM_H2]->logConfig->enabled == true)
			{
				m_LogH2->Write(HapticDataToString(deviceState));
			}
		}

		return deviceState;
	}

    const boost::shared_ptr<HapticManagerConfig> HapticManagerPhantomOmni::GetConfig()
	{
		return m_Config;
	}

} // end namespace Haptics
