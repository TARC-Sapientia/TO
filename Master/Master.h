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


#ifndef MASTER_H
#define MASTER_H

#include <memory>
#include "Core.h"
#include "Connection.h"
#include "CommunicationData.h"
#include "IVideoPlayer.h"
#include "Log.h"
#include "IXmlParser.h"
#include "Master_Controller_Youbot.h"
#include <string>

#ifdef HAPTIC_PHANTOM_OMNI
#include "HapticManagerPhantomOmni.h"
using namespace Haptics;
#endif

#ifdef HAPTIC_NOVINT_FALCON
#include "HapticManagerNovintFalcon.h"
using namespace Haptics;
#endif

using namespace Utils;
using namespace Communication;
using namespace VideoHandler;

/**
 * The master side abstraction layer.
 */
class Master
{
  public:

    /**
     * Constructor
     */
    Master();

    /**
     * Destructor.
     */
    ~Master();

    /**
     * The initializer function.
     * @param[in] configFileName: The name of the configuration file including path to the file.
     */
    void Initialize(string configFileName);

    /**
     * Measures the clock differences between the master and the slave sides.
     */
    void GetClockDifferences();

    /**
     * Starts the main control timer.
     */
    void Start();

  private:

    /**
     * The receive callback function.
     * When a frame is received by a framestream this function is called.
     * If the received message is a control data, than the m_ControlDataReceived is updated.
     * If the received message is a video data, than the frame is queued for display.
     * @param frameStreamID The framestream identifier from which the message is received.
     * @param messageType The type of the received message.
     * @param data The raw message as a byte array.
     * @param dataLength The length of the raw message.
     */
    void OnFrameReceived(uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength);

    /**
     * Starts the main control timer.
     * Based on the availability of the haptic device, either the values are generated.
     * or the the values are retrieved from the device.
     */
    void StartControlTimer();

    /**
     * Generates random control data.
     * Used when no haptic device is connected.
     * @param error_code Parameter passed to the callback function.
     */
    void GenerateControlData(const boost::system::error_code& error_code);

#if defined(HAPTIC_PHANTOM_OMNI) || defined(HAPTIC_NOVINT_FALCON)
    /**
     * Get the state of the haptic device.
     * Fills the master control data with the retreived infromation.
     * Send the master control data through the control frame stream.
     * @param error_code Parameter passed to the callback function.
     */
    void GetControlData(const boost::system::error_code& error_code);
#endif

    /**
     * Sends the control data to the slave.
     * @param controlData The master control data.
     */
    void SendControlData(ControlData controlData);

    /** The control data received in the last time frame. */
    ControlData m_ControlDataReceived;

    /** The haptic data received in the last time frame. */
    ControlData m_PreviousHapticControlData;

    /** Synchronisation object. */
    boost::mutex m_ControlDataMutex;

    /** Flag indicating the presence of the haptic devices. */
    bool m_IsHapticConnected;

    /** The identifier of the control stream. */
    uint32_t m_ControlStreamID;

    /** The main control timer. */
    boost::asio::deadline_timer m_ControlTimer;

#ifdef HAPTIC_PHANTOM_OMNI
    /** The PHANToM Omni haptic manager. */
    boost::shared_ptr<HapticManagerPhantomOmni> m_HapticManagerPhantomOmni;
#endif

#ifdef HAPTIC_NOVINT_FALCON
    /** The Novint Falcon haptic manager. */
    boost::shared_ptr<HapticManagerNovintFalcon> m_HapticManagerNovintFalcon;
#endif

    /** The master side controller of the KUKA youBot. */
    boost::shared_ptr<Master_Controller_Youbot> m_MasterControllerYoubot;

    /** Stores all the configuration parameters of the master side. */
    boost::shared_ptr<MasterConfig> m_Config;

    /** The connection manager used to handle all network communication. */
    boost::shared_ptr<Connection> m_Connection;

    /** The video player interface. Used to display images received from the slave side. */
    VideoHandler::VideoPlayerPtr m_VideoPlayer;

    /** The XML configuration file parser. */
    Utils::XmlParserPtr m_XmlParser;
};

#endif // MASTER_H
