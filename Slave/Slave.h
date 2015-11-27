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



#ifndef SLAVE_H
#define SLAVE_H

#include "Core.h"
#include "Connection.h"
#include "CommunicationData.h"
#include "IVideoCapture.h"
#include "Log.h"
#include "IXmlParser.h"

#ifdef YOUBOT_ON
	#include "YouBotControl.hpp"
#endif

#include <string>

using namespace Communication;
using namespace Utils;

/**
 * The slave side abstraction layer.
 */
class Slave
{
    public:

        /**
        * Constructor
        */
        Slave();

        /**
         * Destructor.
         */
        ~Slave();

        /**
         * The initializer function.
         * @param[in] configFileName: The name of the configuration file including path to the file.
         */
        void Initialize(string configFileName);

        /**
         * Measures the clock differences between the slave and the master sides.
         */
        void GetClockDifferences();

        /**
         * Starts the main control timer.
         */
        void Start();

        #ifdef YOUBOT_ON

            /**
             * Causes the KUKA youBot arm to move in a predefined home position.
             */
            void SendKukatoHomePosition();

            /**
             * Causes the KUKA youBot arm to move in a predefined front position.
             */
            void SendKukaToFrontPosition();

            /**
             * Reads the current state of the KUKA youBot.
             * @param error_code Parameter passed to the callback function.
             */
            void GetControlData(const boost::system::error_code& error_code);

            /** The KUKA youBot manager. */
            boost::shared_ptr<KUKAYouBot::YouBotControl> m_YouBotPtr;

        #endif

    private:
        /**
         * The receive callback function.
         * When a frame is received by a framestream this function is called.
         * If the received message is a control data, than the m_ControlDataReceived is updated.
         * @param frameStreamID The framestream identifier from which the message is received.
         * @param messageType The type of the received message.
         * @param data The raw message as a byte array.
         * @param dataLength The length of the raw message.
         */
        void OnFrameReceived(uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength);

        /**
         * Starts the main control timer.
         * Based on the availability of the KUKA youBot, either the values are generated
         * or the the values are retrieved from the device.
         */
        void StartControlTimer();

        /**
         * Starts the main control timer.
         * Based on the availability of the mounted camera, either the values are generated
         * or the the values are retrieved from the device.
         */
        void StartVideoTimer();

        /**
         * Generates random control data.
         * @param error_code Parameter passed to the callback function.
         */
        void GenerateControlData(const boost::system::error_code& error_code);

        /**
         * Gets the current frame data from the mounted camera.
         * This data is sent through the video stream.
         * @param error_code Parameter passed to the callback function.
         */
        void GetCaptureFrame(const boost::system::error_code& error_code);

        /**
         * Generates random frame data.
         * This generated data is not displayable.
         * @param error_code Parameter passed to the callback function.
         */
        void GenerateCaptureFrame(const boost::system::error_code& error_code);

        /** The control data received in the last time frame. */
        ControlData m_ControlDataReceived;

        /** Synchronization object. */
        boost::mutex m_ControlDataMutex;

        /** The identifier of the control stream. */
        uint32_t m_ControlStreamID;

        /** The identifier of the video stream. */
        uint32_t m_VideoStreamID;

        /** The timer period for getting/generating control data. */
        unsigned int m_ControlTimerPeriod;

        /** The main control timer. */
        boost::asio::deadline_timer m_ControlTimer;

        /** Flag indicating the presence of the mounted camera. */
        bool m_IsCameraConnected;

        /** The video capture control timer. */
        boost::asio::deadline_timer m_VideoTimer;

        /** The timer period of the video capture. */
        unsigned int m_VideoTimerPeriod;

        /** The vidoe capture manager. */
        VideoHandler::VideoCapturePtr m_VideoCapture;

        /** The connection manager used to handle all network communication. */
        boost::shared_ptr<Connection> m_Connection;

        /** The XML configuration file parser. */
        Utils::XmlParserPtr m_XmlParser;

        /** Holds the coordinates of the KUKA youBot's end-effector as a string. */
        std::string m_TextToDisplayOnFrame;

        /** The stream controller mode. */
        Connection::StreamControlMode m_StreamControlMode;
};
#endif // SLAVE_H
