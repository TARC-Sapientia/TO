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


#ifndef I_XML_PARSER_H
#define I_XML_PARSER_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include "IHapticXmlParser.h"
#include "IKukaXmlParser.h"

namespace Utils
{
  /**
   * Structure holding the configurations for Statistics.
   */
  struct StatisticsConfig
  {
      /** The location of the statistics log file. */
      std::string logPath;

      /** The name of the statistics log file. */
      std::string logName;

      /**
       * The size of the statistics window.
       * Represents the number of control packets received after which the statistics are computed.
       */
      unsigned int window;

      /** The coefficient used at the jitter filtering. */
      float alpha;
  };

  /**
   * Structure holding the configurations for a FrameStream.
   */
  struct StreamConfig
  {
      /** The type of the framestream: control, video. */
      std::string type;

      /** The number of the port where incoming frames will be received. */
      unsigned short lport;

      /** The ip of the remote host where outgoing frames will be sent. */
      std::string rhost;

      /** The port used for sending outgoing frames. */
      unsigned short rport;

      /** The size of a chunk. Frames will be split according to this. */
      unsigned short chunkSize;

      /** The size of the receive buffer of the socket. */
      unsigned int receiveBufferSize;

      /** The maximum number of frames queued for sending. */
      unsigned int sendQueueLength;

      /** The configuration of the statistics associated with the stream. */
      boost::shared_ptr<StatisticsConfig> statistics;
  };

  /**
   * Structure holding the configurations for a StreamControl.
   */
  struct StreamControlConfig
  {
      /** Stream control mode: static, active. */
      std::string mode;

      /** The location of the stream control log file. */
      std::string logPath;

      /** The name of the stream control log file. */
      std::string logName;

      /** Structure used to store stream control parameters as key-value pairs. */
      std::map<std::string, std::string> parameters;
  };

  /**
   * Structure holding the configurations for a VideoCapture.
   */
  struct VideoCaptureConfig
  {
      /** The id of the camera. */
      unsigned int camera;

      /** The width of video frames. */
      unsigned int width;

      /** The height of video frames. */
      unsigned int height;

      /** The required frame rate. */
      unsigned short fps;
  };

  /**
   * Structure holding the configurations for remote slaves.
   */
  struct RemoteSlaveConfig
  {
      /** Structure containing multiple FrameStream configurations. */
      std::vector<boost::shared_ptr<StreamConfig> > streams;
  };

  /**
   * Structure that deals with the configuration of a haptic device.
   */
  struct HapticConfig
  {
      /** The location of the haptic config file. */
      std::string configPath;

      /** Data structure to store the haptic configuration parameters. */
      boost::shared_ptr<HapticManagerConfig> hapticManagerConfig;
  };

  /**
   * Structure that deals with the configuration of a KUKA youBot.
   */
  struct KukaConfig
  {
      /** The location of the KUKA youBot config file. */
      std::string configPath;

      /** Data structure to store the KUKA youBot configuration parameters. */
      boost::shared_ptr<KukaManagerConfig> kukaManagerConfig;
  };

  /**
   * Defines how the KUKA youBot is teleoperated.
   */
  enum KukaControllerMode
  {
    /** Phantom Omni is used for teleoperation. The passivity controller is enabled on the base joint of the arm. */
    CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1 = 1,

    /** Phantom Omni is used for teleoperation. The passivity controller is not enabled on the base joint of the arm. */
    CONTROLLER_PHANTOM_KUKA_POSITION_FORCE,

    /** Novint Falcon is used for teleoperation. The passivity controller is enabled on the base joint of the arm. */
    CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1,

    /** Novint Falcon is used for teleoperation. The passivity controller is not enabled on the base joint of the arm. */
    CONTROLLER_NOVINT_KUKA_POSITION_FORCE
  };

  /**
   * Structure holding the configurations for the master.
   */
  struct MasterConfig
  {
      /** Configurations for the remote slaves. */
      boost::shared_ptr<RemoteSlaveConfig> slave;

      /** The number of messages sent in the clock synchronisation phase. */
      unsigned int syncCount;

      /** The constant delay between the messages sent in the clock synchronisation phase. */
      unsigned int syncPeriod;

      /** The period of the control timer. */
      unsigned short controlPeriod;

      /** The size of the frame queue for the new video stream. */
      unsigned int maxDecodingQueueLength;

      /** The configurations for the haptic device. */
      boost::shared_ptr<HapticConfig> haptic;

      /** The configurations for the KUKA youBOT. */
      boost::shared_ptr<KukaConfig> kuka;

      /** Communication delay introduced by the application in "timerPeriod" units; Used to test the passivity controllers. */
      double artificialDelay;

      /** The control mode for the KUKA youBOT. */
      KukaControllerMode kukaControllerMode;

      /** Path to the directory where to log file will be created. */
      std::string logPath;
  };

  /**
   * Structure holding the configurations for the master.
   */
  struct SlaveConfig
  {
      /** Structure containing multiple FrameStream configurations. */
      std::vector<boost::shared_ptr<StreamConfig> > streams;

      /** Configurations for the stream control. */
      boost::shared_ptr<StreamControlConfig> streamControl;

      /** Configurations for the video capture. */
      boost::shared_ptr<VideoCaptureConfig> videoCapture;

      /** The number of messages sent in the clock synchronisation phase. */
      unsigned int syncCount;

      /** The constant delay between the messages sent in the clock synchronisation phase. */
      unsigned int syncPeriod;

      /** The timer period for getting/generating control data. */
      unsigned short controlPeriod;

      /** The timer period of the video capture. */
      unsigned short videoPeriod;

      /** The configurations for the haptic device. */
      boost::shared_ptr<HapticConfig> haptic;

      /** The configurations for the KUKA youBOT. */
      boost::shared_ptr<KukaConfig> kuka;

      /** Communication delay introduced by the application in "timerPeriod" units; Used to test the passivity controllers. */
      double artificialDelay;

      /** The control mode for the KUKA youBOT. */
      KukaControllerMode kukaControllerMode;

      /** Path to the directory where to log file will be created. */
      std::string logPath;
  };

  /**
   * Abstract interface definition for the xml parser.
   */
  class IXmlParser
  {
    public:

      /**
       * Used to parse the slave configuration.
       * @param fileName The file containing the slave configuration.
       * @return a structure containing the parsed configurations.
       */
      virtual boost::shared_ptr<SlaveConfig> ParseSlaveConfig(std::string fileName) = 0;

      /**
       * Used to parse the master configuration.
       * @param fileName The file containing the master configuration.
       * @return a structure containing the parsed configurations.
       */
      virtual boost::shared_ptr<MasterConfig> ParseMasterConfig(std::string fileName) = 0;
  };

  /**
   * Smart pointer wrapper.
   */
  template <class T>
  class IXmlParserPtr : public boost::shared_ptr<T>
  {
    public:
      /**
       * Constructor.
       */
      IXmlParserPtr(T* p) : boost::shared_ptr<T>(p) { }
  };

  /** Smart pointer type definition. */
  typedef IXmlParserPtr<IXmlParser> XmlParserPtr;  // create an alias for specialized smart pointer

  /**
   * Used to create an xml parser instance.
   * @return the pointer to the newly created xml parser.
   */
  IXmlParser* getXmlParser();

} // end namespace Utils

#endif //I_XML_PARSER_H
