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


#include "Master.h"
#include "Utilities.h"
#include "VideoPlayback.h"
#include <boost/lexical_cast.hpp>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>

using namespace std;
using namespace boost;
using namespace Utils;
using namespace Communication;

Master::Master():
  m_ControlTimer(Core::get().getIoService()),
  m_VideoPlayer(VideoHandler::getVideoPlayer()),
  m_IsHapticConnected(false),
  m_XmlParser(Utils::getXmlParser())
{
  memset(&m_ControlDataReceived, 0, sizeof(m_ControlDataReceived));
  memset(&m_PreviousHapticControlData, 0, sizeof(m_PreviousHapticControlData));
}

void Master::Initialize(string configFileName)
{
  m_Config = m_XmlParser->ParseMasterConfig(configFileName);

  // Create the connection
  m_Connection.reset(new Connection(m_Config->syncCount, m_Config->syncPeriod));
  m_Connection->SetReceiveFunction(boost::bind(&Master::OnFrameReceived, this, _1, _2, _3, _4));

  // Iterate over the remote slave streams
  vector<boost::shared_ptr<StreamConfig> >::iterator it;
  for (it = m_Config->slave->streams.begin(); it != m_Config->slave->streams.end(); ++it)
  {
    if (!strcmp(it->get()->type.c_str(), "video"))
    {
      unsigned int videoStreamID = m_Connection->AddFrameStream(VideoStream, it->get()->lport, it->get()->rhost, it->get()->rport, it->get()->chunkSize,
                                                                it->get()->receiveBufferSize, it->get()->sendQueueLength, m_Config->controlPeriod, it->get()->statistics);

      m_VideoPlayer->addVideoPlayback(videoStreamID, m_Config->maxDecodingQueueLength);
    }
    else if (!strcmp(it->get()->type.c_str(), "control"))
    {
      m_ControlStreamID = m_Connection->AddFrameStream(ControlStream, it->get()->lport, it->get()->rhost, it->get()->rport, it->get()->chunkSize,
                                                       it->get()->receiveBufferSize, it->get()->sendQueueLength, m_Config->controlPeriod, it->get()->statistics);
    }

    cout << "Connected to " << it->get()->type << " on: " << it->get()->rhost << ":" << it->get()->rport << endl;
  }

#ifdef HAPTIC_PHANTOM_OMNI
  m_HapticManagerPhantomOmni.reset(new HapticManagerPhantomOmni(m_Config->haptic->configPath));

  // At least on of the devices is connected.
  m_IsHapticConnected = m_HapticManagerPhantomOmni->IsConnected(PHANToM_H1) || m_HapticManagerPhantomOmni->IsConnected(PHANToM_H2);

  if (m_IsHapticConnected == false)
  {
    cout << "No haptic device connected. Generating data." << endl;
  }
#endif

#ifdef HAPTIC_NOVINT_FALCON
  m_HapticManagerNovintFalcon.reset(new HapticManagerNovintFalcon(m_Config->haptic->configPath));

  m_HapticManagerNovintFalcon->Initialize();

  // At least on of the devices is connected.
  m_IsHapticConnected = m_HapticManagerNovintFalcon->IsConnected(NOVINT_H1) || m_HapticManagerNovintFalcon->IsConnected(NOVINT_H2);

  if (m_IsHapticConnected && m_HapticManagerNovintFalcon->IsCalibrated(NOVINT_H1) == false)
  {
    cout << "Warning NOVINT_H1 not calibrated! Please home the device by extending then pushing the arms all the way in!" << endl;
    cout << "Press ENTER to continue." << endl;
    cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
  }

  if (m_IsHapticConnected == false)
  {
    cout << "No haptic device connected. Generating data." << endl;
  }
#endif

  // Reads the config parameters
  if (m_Config->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE)
  {
    cout << "Controller Mode: CONTROLLER_NOVINT_KUKA_POSITION_FORCE." << endl;
  }
  else if (m_Config->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1)
  {
    cout << "Controller Mode: CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1." << endl;
  }
  else if (m_Config->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE)
  {
    cout << "Controller Mode: CONTROLLER_PHANTOM_KUKA_POSITION_FORCE." << endl;
  }
  else if (m_Config->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1)
  {
    cout << "Controller Mode: CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1." << endl;
  }

  bool passivityControlEnabled = false;

  if(m_Config->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1 ||
     m_Config->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1 )
  {
    passivityControlEnabled = true;

#ifdef HAPTIC_PHANTOM_OMNI
    m_MasterControllerYoubot.reset(
        new Master_Controller_Youbot(
            m_Config->logPath,
            (double)m_Config->controlPeriod / 1000.0, // msec to sec
            m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->constraintsConfig->maxForce,
            m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->constraintsConfig->maxVelocity,
            passivityControlEnabled,
            (unsigned int)m_Config->artificialDelay,
            m_Config->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint1,
            m_Config->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint1,
            m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->constraintsConfig->minJoint1,
            m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->constraintsConfig->maxJoint1,
            m_Config->kuka->kukaManagerConfig->controllerParametersConfig->integralGainVelocityControl
    ));
#endif

#ifdef HAPTIC_NOVINT_FALCON
    m_MasterControllerYoubot.reset(
        new Master_Controller_Youbot(
            m_Config->logPath,
            (double)m_Config->controlPeriod / 1000.0, // msec to sec
            m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->constraintsConfig->maxForce,
            m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->constraintsConfig->maxVelocity,
            passivityControlEnabled,
            (unsigned int)m_Config->artificialDelay,
            m_Config->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint1,
            m_Config->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint1,
            m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->constraintsConfig->minJoint1,
            m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->constraintsConfig->maxJoint1,
            m_Config->kuka->kukaManagerConfig->controllerParametersConfig->integralGainVelocityControl
    ));
#endif
  }
}

Master::~Master()
{
  m_ControlTimer.cancel();

  cout<<"Master stopped." << endl;
}

void Master::Start()
{
  StartControlTimer();
}

void Master::GetClockDifferences() 
{	
  m_Connection->CalculateClockDifferences();
}

void Master::OnFrameReceived(uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength)
{
  if (messageType == MessageType_CONTROL)
  {
    m_ControlDataMutex.lock();
    memcpy(&m_ControlDataReceived, data.get(), dataLength);
    m_ControlDataMutex.unlock();
  }
  else if (messageType == MessageType_VIDEO)
  {
    m_VideoPlayer->pushEncodedFrame(frameStreamID, data, dataLength);
  }
}

void Master::StartControlTimer()
{
  m_ControlTimer.expires_from_now(boost::posix_time::milliseconds(m_Config->controlPeriod));

#ifdef HAPTIC_OFF
  m_ControlTimer.async_wait(boost::bind(&Master::GenerateControlData, this, boost::asio::placeholders::error));
#else
  if (m_IsHapticConnected == true)
  {
    m_ControlTimer.async_wait(boost::bind(&Master::GetControlData, this, boost::asio::placeholders::error));
  }
  else
  {
    // Want to use the device but it is not connected.
    m_ControlTimer.async_wait(boost::bind(&Master::GenerateControlData, this, boost::asio::placeholders::error));
  }
#endif
}

void PrintControlData(ControlData& controlData)
{
  static int szamlalo = 0;
  szamlalo++;
  if (szamlalo > 50)
  {
    szamlalo = 0;
    PrintVector("Position", controlData.position, controlDataCols);
    PrintVector("Velocity", controlData.velocity, controlDataCols);
    PrintVector("Force", controlData.force, controlDataCols);
    PrintVector("Energy",controlData.energy, controlDataCols);
    cout << " ------------------------------------------" << endl;
  }
}

void Master::GenerateControlData(const boost::system::error_code& error_code)
{
  if(!error_code)
  {
    StartControlTimer();

    ControlData received = m_ControlDataReceived;
    m_ControlDataMutex.lock();
    memcpy(&received, &m_ControlDataReceived, sizeof(m_ControlDataReceived));
    m_ControlDataMutex.unlock();

    PrintControlData(received);

    ControlData response;
    memset(&response, 0, sizeof(response));

    unsigned int incValue = 1; // Value to be sent

    for(int i = 0; i < controlDataCols; i++)
    {
      response.position[i] = incValue++;
      response.velocity[i] = incValue++;
      response.force[i] = incValue++;
    }

    int dataLength = sizeof(response);
    char *pData = new char[dataLength];
    memcpy(pData, &response, dataLength);

    boost::shared_array<char> data(pData);
    m_Connection->WriteFrame(m_ControlStreamID, MessageType_CONTROL, data, dataLength);
  }
}

#if defined(HAPTIC_NOVINT_FALCON)
void Master::GetControlData(const boost::system::error_code& error_code)
{
  if(!error_code)
  {
    StartControlTimer();

    ControlData received;
    m_ControlDataMutex.lock();
    memcpy(&received, &m_ControlDataReceived, sizeof(m_ControlDataReceived));
    m_ControlDataMutex.unlock();

    ControlData response; memset(&response, 0, sizeof(response));

    vector<double> h1_position = m_HapticManagerNovintFalcon->GetPosition(NOVINT_H1);
    vector<double> h1_velocity = m_HapticManagerNovintFalcon->GetVelocity(NOVINT_H1);
    vector<double> h1_force = m_HapticManagerNovintFalcon->GetForce(NOVINT_H1);

    std::vector<double> force_to_Haptic(3, 0.0);

    // Sets the force for the haptic device
    if (m_Config->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE)
    {
      double scale_HX = m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->scallingCoefficientForceFeedback[0];
      double scale_HY = m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->scallingCoefficientForceFeedback[1];
      double scale_HZ = m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->scallingCoefficientForceFeedback[2];

      double velocityThreshold = m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->thresholdForceFeedbackConfig->velocityThreshold;
      double forceThreshold = m_Config->haptic->hapticManagerConfig->devices[NOVINT_H1]->thresholdForceFeedbackConfig->forceThreshold;

      // Avoids the high force values during the acceleration of a robot joint
      if (abs((received.velocity[3]) > velocityThreshold) && (received.force[3] > forceThreshold))
      {
        received.force[3] = forceThreshold;
      }
      if (abs((received.velocity[3]) > velocityThreshold) && (received.force[3] < -forceThreshold))
      {
        received.force[3] = -forceThreshold;
      }
      force_to_Haptic[0] = -scale_HX * received.force[3];

      if (abs((received.velocity[4]) > velocityThreshold) && (received.force[4] > forceThreshold))
      {
        received.force[4] = forceThreshold;
      }
      if (abs((received.velocity[4]) > velocityThreshold) && (received.force[4] < -forceThreshold))
      {
        received.force[4] = -forceThreshold;
      }
      force_to_Haptic[1] = - scale_HY * received.force[4];

      if (abs((received.velocity[5]) > velocityThreshold) && (received.force[5] > forceThreshold))
      {
        received.force[5] = forceThreshold;
      }
      if (abs((received.velocity[5]) > velocityThreshold) && (received.force[5] < -forceThreshold))
      {
        received.force[5] = -forceThreshold;
      }
      force_to_Haptic[2] = - scale_HZ * received.force[5];

      if (m_HapticManagerNovintFalcon->IsButtonPressed(NOVINT_H1, ButtonRear) == true)
      {
        m_HapticManagerNovintFalcon->SetForce(NOVINT_H1, force_to_Haptic);
      }
      else
      {
        m_HapticManagerNovintFalcon->SetForce(NOVINT_H1, std::vector<double>(3, 0.0));
      }
    }
    else  // PO-PC enabled for KUKA base joint
    {
      ControlData dataToMasterController;
      memset(&dataToMasterController, 0, sizeof(dataToMasterController));

	  dataToMasterController.position[3] = h1_position[0];
      dataToMasterController.position[4] = h1_position[1];
      dataToMasterController.position[5] = h1_position[2];

      dataToMasterController.velocity[3] = h1_velocity[0];
      dataToMasterController.velocity[4] = h1_velocity[1];
      dataToMasterController.velocity[5] = h1_velocity[2];

      dataToMasterController.force[3] = h1_force[0];
      dataToMasterController.force[4] = h1_force[1];
      dataToMasterController.force[5] = h1_force[2];

      m_MasterControllerYoubot->SetCurrentHapticControlData(dataToMasterController);
      m_MasterControllerYoubot->SetCurrentSlaveControlData(received);

      ControlData passifiedData = m_MasterControllerYoubot->GetNewHapticControlData();

      if (m_HapticManagerNovintFalcon->IsButtonPressed(NOVINT_H1, ButtonRear) == true) // If KUKA arm is enabled
      {
        force_to_Haptic[0] = passifiedData.force[3] * (-1);
        force_to_Haptic[1] = passifiedData.force[4] * (-1);
        force_to_Haptic[2] = passifiedData.force[5] * (-1);
        m_HapticManagerNovintFalcon->SetForce(NOVINT_H1, force_to_Haptic);
      }

      response = passifiedData;
    }

    // Builds the response data
    response.position[0] = -h1_position[2];		// h1 Z
    response.position[1] = -h1_position[0];		// h1 X
    response.position[2] = h1_position[1];		// h1 Y
    response.position[3] = h1_position[0];		// h1 X -> J1
    response.position[4] = -h1_position[2];		// h1 Z -> J2
    response.position[5] = -h1_position[1];		// h1 Y -> J3
    response.position[6] = 0.0;					// constant -> J4
    response.position[7] = 0.0;					// constant -> J5

    response.velocity[0] = -h1_position[2];		// h1 vz -> vlong
    response.velocity[1] = -h1_position[0];		// h1 vx -> vlat
    response.velocity[2] = h1_position[1];		// h1 vy -> w
    response.velocity[3] = h1_velocity[0];		// h1 vx
    response.velocity[4] = -h1_velocity[2];		// h1 vz
    response.velocity[5] = h1_velocity[1];		// h1 vy

    response.force[0] = -h1_force[2];		// h1 fz
    response.force[1] = h1_force[0];		// h1 fx
    response.force[2] = h1_force[1];		// h1 fy
    response.force[3] = h1_force[0];		// h1 fx
    response.force[4] = -h1_force[2];		// h1 fz
    response.force[5] = h1_force[1];		// h1 fy

    if (m_HapticManagerNovintFalcon->IsButtonPressed(NOVINT_H1, ButtonFront) == true)
    {
      response.force[8] = 2;		// Enable platform motion
    }
    if (m_HapticManagerNovintFalcon->IsButtonPressed(NOVINT_H1, ButtonRear) == true)
    {
      response.force[8] = 1;		// Enable arm motion
    }
    if (m_HapticManagerNovintFalcon->IsButtonPressed(NOVINT_H1, ButtonRight) == true)
    {
      response.force[8] = 3;		// Change gripper state.
    }

    // PrintControlData(received);
    // PrintControlData(response);

    SendControlData(response);
  }
}
#endif 

#if defined(HAPTIC_PHANTOM_OMNI) 
void Master::GetControlData(const boost::system::error_code& error_code)
{
  if(!error_code)
  {
    StartControlTimer();

    ControlData received;
    m_ControlDataMutex.lock();
    memcpy(&received, &m_ControlDataReceived, sizeof(m_ControlDataReceived));
    m_ControlDataMutex.unlock();

    ControlData response;
    memset(&response, 0, sizeof(response));

    HapticData h1 = m_HapticManagerPhantomOmni->GetData(PHANToM_H1);
    HapticData h2 = m_HapticManagerPhantomOmni->GetData(PHANToM_H2);

    double force_to_Haptic[3] = {0};
    double mm_to_m = 1.0 / 1000.0;

    if (m_Config->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE)
    {
      // Sets the force fort the haptic device
      double scale_HJ0 = m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->scallingCoefficientForceFeedback[0];
      double scale_HJ1 = m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->scallingCoefficientForceFeedback[1];
      double scale_HJ2 = m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->scallingCoefficientForceFeedback[2];
      double velocityThreshold = m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->thresholdForceFeedbackConfig->velocityThreshold;
      double forceThreshold = m_Config->haptic->hapticManagerConfig->devices[PHANToM_H1]->thresholdForceFeedbackConfig->forceThreshold;

      // Avoids the high force values during the acceleration of a robot joint
      if ((abs((received.velocity[3]) > velocityThreshold) && (received.force[3] > forceThreshold)))
      {
        received.force[3] = forceThreshold;
      }
      if ((abs((received.velocity[3]) > velocityThreshold) && (received.force[3] < -forceThreshold)))
      {
        received.force[3] = -forceThreshold;
      }
      force_to_Haptic[0] = scale_HJ0 * received.force[3];

      if ((abs((received.velocity[4]) > velocityThreshold) && (received.force[4] > forceThreshold)))
      {
        received.force[4] = forceThreshold;
      }
      if ((abs((received.velocity[4]) > velocityThreshold) && (received.force[4] < -forceThreshold)))
      {
        received.force[4] = -forceThreshold;
      }
      force_to_Haptic[1] = - scale_HJ1 * received.force[4];

      if ((abs((received.velocity[5]) > velocityThreshold) && (received.force[5] > forceThreshold)))
      {
        received.force[5] = forceThreshold;
      }
      if ((abs((received.velocity[5]) > velocityThreshold) && (received.force[5] < -forceThreshold)))
      {
        received.force[5] = -forceThreshold;
      }
      force_to_Haptic[2] = - scale_HJ2 * received.force[5];

      if (h1.buttonState == DarkPressed)
      {
        m_HapticManagerPhantomOmni->SetForce(PHANToM_H1, force_to_Haptic);
      }
    }
    else // PO-PC enabled for KUKA base joint
    {
	  received.force[3] *= -1;

      ControlData dataToMasterController = m_PreviousHapticControlData;

	  dataToMasterController.position[3] = h1.position[0] * mm_to_m;
      dataToMasterController.position[4] = h1.position[1] * mm_to_m;
      dataToMasterController.position[5] = h1.position[2] * mm_to_m;

      dataToMasterController.velocity[3] = h1.velocity[0] * mm_to_m;
      dataToMasterController.velocity[4] = h1.velocity[1] * mm_to_m;
      dataToMasterController.velocity[5] = h1.velocity[2] * mm_to_m;

      m_MasterControllerYoubot->SetCurrentHapticControlData(dataToMasterController);
      m_MasterControllerYoubot->SetCurrentSlaveControlData(received);

      ControlData passifiedData = m_MasterControllerYoubot->GetNewHapticControlData();
      m_PreviousHapticControlData = passifiedData;

      if (h1.buttonState == DarkPressed) // If KUKA arm is enabled
      {
        force_to_Haptic[0] = passifiedData.force[3] * (-1);
        force_to_Haptic[1] = passifiedData.force[4] * (-1);
        force_to_Haptic[2] = passifiedData.force[5] * (-1);
        m_HapticManagerPhantomOmni->SetForce(PHANToM_H1, force_to_Haptic);
      }

      response = passifiedData;
    }

    // Same response for both control modes
    response.force[3] = force_to_Haptic[0];
    response.force[4] = force_to_Haptic[1];
    response.force[5] = force_to_Haptic[2];

    response.velocity[0] = -h2.position[2];		// Z of haptic [mm] -> longitudinal of KUKA platform
    response.velocity[1] = -h2.position[0];		// X of haptic [mm] -> transversal of KUKA platform
    response.velocity[2] = h2.gimbalAngles[0];  // First gimbal angle of haptic [rad] -> angular of KUKA platform

    response.velocity[3] = h1.velocity[0] * mm_to_m;
    response.velocity[4] = h1.velocity[1] * mm_to_m;
    response.velocity[5] = h1.velocity[2] * mm_to_m;

    response.position[3] = h1.jointAngles[0];
    response.position[4] = h1.jointAngles[1];
    response.position[5] = h1.jointAngles[2];

    response.position[6] = h1.gimbalAngles[0];
    response.position[7] = h1.gimbalAngles[1];
    response.position[8] = h1.gimbalAngles[2];

    response.force[8] = h1.buttonState; // Switched devices. H2 buttons are not working.

    // PrintControlData(response);
    PrintControlData(received);

    SendControlData(response);
  }
}
#endif

void Master::SendControlData(ControlData controlData)
{
  // Should use controlData.ToString() method. Instead of memcpy
  // Sends the data to slave
  int dataLength = sizeof(controlData);
  char *pData = new char[dataLength];
  memcpy(pData, &controlData, dataLength);

  boost::shared_array<char> data(pData);
  m_Connection->WriteFrame(m_ControlStreamID, MessageType_CONTROL, data, dataLength);
}
