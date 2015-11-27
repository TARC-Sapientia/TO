#include "HapticManagerNovintFalcon.h"
#include <iostream>
#include <math.h>
#include <string>
#include <boost\thread.hpp>
#include "Utilities.h"

#include <hdl/hdl.h>
#include <hdlu/hdlu.h> 
using namespace std;
using namespace Utils;

namespace Haptics
{
	// Variables used only by servo thread 
	double currentPosition[3];
	double currentForce[3];
	int currentButtons;

    bool isDeviceConnected_H1;

    string deviceName_H1;

    HDLDeviceHandle deviceHandle_H1;
	
	int HDLSchedulerHandle;
	
	boost::recursive_mutex mutex;

	// Continuous servo callback function.
	HDLServoOpExitCode AsyncSchedulerCallback(void* pUserData)
	{
		// Make the device current. All subsequent calls will be directed towards the current device. 
		hdlMakeCurrent(deviceHandle_H1);

		mutex.lock();

		// Get current state of haptic device
		hdlToolPosition(currentPosition);
		hdlToolButtons(&currentButtons);

		// Set current force 
		hdlSetToolForce(currentForce);		
		
        mutex.unlock();

		// Make sure to continue processing 
		return HDL_SERVOOP_CONTINUE;
	}

	HapticManagerNovintFalcon::HapticManagerNovintFalcon(std::string configFileName)
		: m_T_k(0), m_T_k_1(0),
		m_Position_k_1(3, 0.0),
		m_Velocity_k_1(3, 0.0)
	{
        deviceName_H1 = "DEFAULT";

		memset(currentPosition, 0, sizeof(currentPosition));
		memset(currentForce, 0, sizeof(currentForce));

		HapticXmlParserPtr xmlParser = getHapticXmlParser();
		
		try
		{
			m_Config = xmlParser->ParseHapticManagerConfig(configFileName);
		}		
		catch (exception* e)
		{
			cout << "Haptic Manager: Failed to parse configuration file: " << configFileName << endl;
			cout << e->what() << endl;
		}
	}

	HapticManagerNovintFalcon::~HapticManagerNovintFalcon()
	{
		CleanUp();
	}

	bool HapticManagerNovintFalcon::Initialize()
	{
        isDeviceConnected_H1 = false;

		if ((deviceHandle_H1 = hdlInitNamedDevice(deviceName_H1.c_str())) == HDL_INVALID_HANDLE)
		{
			cout << "Haptic Manager: Device: " << deviceName_H1 << " is NOT connected." << endl;

			return false;
		}
		else
		{
			isDeviceConnected_H1 = true;
		}

		// Now that the device is fully initialized, start the servo thread.
		// Failing to do this will result in a non-funtional haptics application.
		hdlStart();
		
        const bool bNonBlocking = false;

		// Set up callback function
		if ((HDLSchedulerHandle = hdlCreateServoOp(AsyncSchedulerCallback, this, bNonBlocking)) == HDL_INVALID_HANDLE)
		{
			cout << "Haptic Manager: Invalid servo op handle!" << endl;
			return false;
		}

		return true;
	}
	
	void HapticManagerNovintFalcon::CleanUp()
	{
		if (HDLSchedulerHandle != HDL_INVALID_HANDLE)
		{
			hdlDestroyServoOp(HDLSchedulerHandle);
			HDLSchedulerHandle = HDL_INVALID_HANDLE;
		}

		hdlStop();

		if (deviceHandle_H1 != HDL_INVALID_HANDLE)
		{
			hdlUninitDevice(deviceHandle_H1);
			deviceHandle_H1 = HDL_INVALID_HANDLE;
        }
	}

	bool HapticManagerNovintFalcon::IsConnected(HapticDevice device)
	{
        if (device == NOVINT_H1) return isDeviceConnected_H1;

		return false;
	}

	bool HapticManagerNovintFalcon::IsButtonPressed(HapticDevice device, Button button)
    {
		if (button == ButtonFront) return currentButtons == HDL_BUTTON_3;
		if (button == ButtonLeft) return currentButtons == HDL_BUTTON_2;
		if (button == ButtonRear) return currentButtons == HDL_BUTTON_1;
		if (button == ButtonRight) return currentButtons == HDL_BUTTON_4;

		return false;
	}

	std::vector<double> HapticManagerNovintFalcon::GetPosition(HapticDevice device)
	{
		vector<double> pos; 

		mutex.lock();
		pos.assign(currentPosition, currentPosition + 3);	
		mutex.unlock();

		return pos;
	}

	std::vector<double> HapticManagerNovintFalcon::GetVelocity(HapticDevice device)
    {
		vector<double> velocity(3, 0.0);

		m_T_k = clock();

		vector<double> position_k = GetPosition(device); 

		if(m_T_k_1 != 0)
		{
			double DT = (double)(m_T_k - m_T_k_1) / CLOCKS_PER_SEC;
			
			double vx_k = 0;
			double vy_k = 0;
			double vz_k = 0;

			if (DT != 0)
			{
				vx_k = (position_k[0] - m_Position_k_1[0]) / DT;
				vy_k = (position_k[1] - m_Position_k_1[1]) / DT;
				vz_k = (position_k[2] - m_Position_k_1[2]) / DT;
			}

			velocity[0] = m_Config->devices[NOVINT_H1]->constraintsConfig->velocityAlphaFilter * m_Velocity_k_1[0] + (1 - m_Config->devices[NOVINT_H1]->constraintsConfig->velocityAlphaFilter) * vx_k;
			velocity[1] = m_Config->devices[NOVINT_H1]->constraintsConfig->velocityAlphaFilter * m_Velocity_k_1[1] + (1 - m_Config->devices[NOVINT_H1]->constraintsConfig->velocityAlphaFilter) * vy_k;
			velocity[2] = m_Config->devices[NOVINT_H1]->constraintsConfig->velocityAlphaFilter * m_Velocity_k_1[2] + (1 - m_Config->devices[NOVINT_H1]->constraintsConfig->velocityAlphaFilter) * vz_k;
			
			m_Velocity_k_1 = velocity;
		}

		m_Position_k_1 = position_k;
		m_T_k_1 = m_T_k;
		return velocity;
	}

	std::vector<double> HapticManagerNovintFalcon::GetWorkspaceDimensions(HapticDevice device)
	{
		double workspaceDims[6]; 
		memset(workspaceDims, 0, sizeof(workspaceDims));
		hdlDeviceWorkspace(workspaceDims);

		vector<double> workspace;
		workspace.assign(workspaceDims, workspaceDims + 6);	

		return workspace;
	}

	void HapticManagerNovintFalcon::SetForce(HapticDevice device, std::vector<double> force)
	{
		
		if (force[0] > m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce) force[0] = m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce;
		if (force[1] > m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce) force[1] = m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce;
		if (force[2] > m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce) force[2] = m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce;
				
		if (force[0] < -m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce) force[0] = -m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce;
		if (force[1] < -m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce) force[1] = -m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce;
		if (force[2] < -m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce) force[2] = -m_Config->devices[NOVINT_H1]->constraintsConfig->maxForce;

		mutex.lock();
		currentForce[0] = force[0];
		currentForce[1] = force[1];
		currentForce[2] = force[2];

		mutex.unlock();
	}

	std::vector<double> HapticManagerNovintFalcon::GetForce(HapticDevice device)
	{
		vector<double> force; 

		mutex.lock();
		force.assign(currentForce, currentForce + 3);	
		mutex.unlock();

		return force;
	}

	bool HapticManagerNovintFalcon::IsCalibrated(HapticDevice device)
    {
		return 0 == (hdlGetState() & HDAL_NOT_CALIBRATED);
	}
}
