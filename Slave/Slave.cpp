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


#include "Slave.h"
#include "Utilities.h"
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace Utils;

#ifdef YOUBOT_ON
    using namespace KUKAYouBot;
#endif

Slave::Slave() :
     m_ControlTimer(Core::get().getIoService()),
     m_VideoTimer(Core::get().getIoService()),
     m_VideoCapture(VideoHandler::getVideoCapture()),
     m_XmlParser(Utils::getXmlParser())
{
}

void Slave::Initialize(string configFileName)
{
    boost::shared_ptr<SlaveConfig> config = m_XmlParser->ParseSlaveConfig(configFileName);

    // Create the connection
    m_Connection.reset(new Connection(config->syncCount, config->syncPeriod));
    m_Connection->SetReceiveFunction(boost::bind(&Slave::OnFrameReceived, this, _1, _2, _3, _4));

    m_VideoTimerPeriod = config->videoPeriod;
    m_ControlTimerPeriod = config->controlPeriod;

    // Create the slave stream control
    m_Connection->CreateStreamControl(config->streamControl);
    m_StreamControlMode = m_Connection->GetStreamControlMode();
    
    #ifdef YOUBOT_ON
        m_YouBotPtr.reset(new YouBotControl(config));
        m_YouBotPtr->InitRobotBase();
        m_YouBotPtr->InitRobotManipulator();

        if (config->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE)
        {
            cout << "Controller Mode: CONTROLLER_NOVINT_KUKA_POSITION_FORCE." << endl;
        }
        else if (config->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1)
        {
            cout << "Controller Mode: CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1." << endl;
        }
        else if (config->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE)
        {
            cout << "Controller Mode: CONTROLLER_PHANTOM_KUKA_POSITION_FORCE." << endl;
        }
        else if (config->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1)
        {
            cout << "Controller Mode: CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1." << endl;
        }
    #endif

    // Iterate over the streams
    vector<boost::shared_ptr<StreamConfig> >::iterator it;
    for (it = config->streams.begin(); it != config->streams.end(); ++it) {
        if (!strcmp(it->get()->type.c_str(), "video")) {
            m_VideoStreamID = m_Connection->AddFrameStream(VideoStream, it->get()->lport, it->get()->rhost, it->get()->rport, it->get()->chunkSize, 
                it->get()->receiveBufferSize, it->get()->sendQueueLength, config->controlPeriod, it->get()->statistics);
        }
        else if (!strcmp(it->get()->type.c_str(), "control")) {
            m_ControlStreamID = m_Connection->AddFrameStream(ControlStream, it->get()->lport, it->get()->rhost, it->get()->rport, it->get()->chunkSize, 
                it->get()->receiveBufferSize, it->get()->sendQueueLength, config->controlPeriod, it->get()->statistics);
        }

        cout << "Connected to " << it->get()->type << " on: " << it->get()->rhost << ":" << it->get()->rport << endl;
    }

    if (!m_VideoCapture->initCapture(config->videoCapture->camera, config->videoCapture->fps)) {
        cout << "Cannot open video capture! Generating video data.." << endl;
        m_IsCameraConnected = false;
    }
    else {
        m_VideoCapture->setCaptureResolution(config->videoCapture->width, config->videoCapture->height);
        m_IsCameraConnected = true;
    }

    memset(&m_ControlDataReceived, 0, sizeof(m_ControlDataReceived));
}

Slave::~Slave()
{
    m_ControlTimer.cancel();
    m_VideoTimer.cancel();

    Core::get().stop();

    #ifdef YOUBOT_ON
        SendKukatoHomePosition();

        // Due to speed limitation the arm does not reach the home position
        SLEEP_MILLISEC(10000);
    #endif
}

void Slave::Start()
{
    StartControlTimer();
    StartVideoTimer();
}

void Slave::GetClockDifferences()
{
    m_Connection->CalculateClockDifferences();
}

void Slave::OnFrameReceived(uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength)
{
    if (messageType == MessageType_CONTROL)
    {
        m_ControlDataMutex.lock();
        memcpy(&m_ControlDataReceived, data.get(), dataLength);
        m_ControlDataMutex.unlock();
    }
}

void Slave::StartControlTimer()
{
    m_ControlTimer.expires_from_now(boost::posix_time::milliseconds(m_ControlTimerPeriod));

	#ifdef YOUBOT_ON
        m_ControlTimer.async_wait(boost::bind(&Slave::GetControlData, this, boost::asio::placeholders::error));
    #else
		m_ControlTimer.async_wait(boost::bind(&Slave::GenerateControlData, this, boost::asio::placeholders::error));    	
    #endif
}

void Slave::StartVideoTimer()
{
    m_VideoTimer.expires_from_now(boost::posix_time::milliseconds(m_VideoTimerPeriod));
    
    if (m_IsCameraConnected)
    {
        m_VideoTimer.async_wait(boost::bind(&Slave::GetCaptureFrame, this, boost::asio::placeholders::error));
    }
    else
    {
        m_VideoTimer.async_wait(boost::bind(&Slave::GenerateCaptureFrame, this, boost::asio::placeholders::error));
    }
}

void Slave::GetCaptureFrame(const boost::system::error_code& error_code)
{
    if (!error_code)
    {
        StartVideoTimer();
        boost::shared_array<char> data;

        int x, y = 0;
        m_ControlDataMutex.lock();
        x = static_cast<int>((80 + m_ControlDataReceived.position[0]) * 640 / 160);
        y = static_cast<int>((60 - m_ControlDataReceived.position[1]) * 480 / 120);
        m_ControlDataMutex.unlock();

        double compression = m_Connection->GetLocalCompression();

        int size = m_VideoCapture->captureJPEG(data, (unsigned short)compression, x, y, m_TextToDisplayOnFrame);
        
        if (size > 0)
        {
            m_Connection->WriteFrame(m_VideoStreamID, MessageType_VIDEO, data, size);
        }
    }
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
        PrintVector("Energy", controlData.energy, controlDataCols);
        cout << " ------------------------------------------" << endl;
    }
}

void Slave::GenerateCaptureFrame(const boost::system::error_code& error_code)
{
    if(!error_code)
    {
        StartVideoTimer();

        string str(40 * KILO_BYTE, 'z');
        char *pData = new char[str.length()];

        memcpy(pData, str.c_str(), str.length());

        boost::shared_array<char> data(pData);

        m_Connection->WriteFrame(m_VideoStreamID, MessageType_VIDEO, data, str.length());        
    }
}

void Slave::GenerateControlData(const boost::system::error_code& error_code)
{
    if(!error_code)
    {
        StartControlTimer();

        ControlData received = m_ControlDataReceived;
        m_ControlDataMutex.lock();
        memcpy(&received, &m_ControlDataReceived, sizeof(m_ControlDataReceived));
        m_ControlDataMutex.unlock();

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

		PrintControlData(received);
    }
}

#ifdef YOUBOT_ON

void Slave::GetControlData(const boost::system::error_code& error_code)
{
    if (!error_code)
    {
        // Receives Data from Master
        ControlData receivedControlData;
        m_ControlDataMutex.lock();
        memcpy(&receivedControlData, &m_ControlDataReceived, sizeof(m_ControlDataReceived));
        m_ControlDataMutex.unlock();

        // Generates Data for Master
        ControlData responseData;
        memset(&responseData, 0, sizeof(responseData));

        responseData = m_YouBotPtr->ProcessInputData(receivedControlData);

        // Builds the end-effector position string for display
        stringstream ss;
        ss << fixed << setprecision(3)
            << "X: " << setw(8) << responseData.position[8]
            << " Y: " << setw(8) << responseData.velocity[8]
            << " Z:" << setw(8) << responseData.force[8];
        m_TextToDisplayOnFrame = ss.str();

        //PrintControlData(responseData);
        PrintControlData(receivedControlData);

        // Sends Data to Master
        uint32_t dataLength = sizeof(responseData);
        char *pData = new char[dataLength];
        memcpy(pData, &responseData, dataLength);

        boost::shared_array<char> data(pData);
        m_Connection->WriteFrame(m_ControlStreamID, MessageType_CONTROL, data, dataLength);
        
        StartControlTimer();
    }
}

void Slave::SendKukatoHomePosition()
{
    m_YouBotPtr->SendArmToHomePosition();
}
void Slave::SendKukaToFrontPosition()
{
    m_YouBotPtr->SendArmToFrontPosition();
}
#endif
