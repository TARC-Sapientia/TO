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


#pragma once
#ifndef Connection_h
#define Connection_h

#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif
#endif // _WIN32

#include <string>
#include <vector>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/shared_array.hpp>
#include <IXmlParser.h>
#include "StreamControl.h"
#include "Log.h"

using namespace std;

namespace Communication
{
  class FrameStream;

  /** Enum for possible stream types. */
  enum StreamType
  {
    VideoStream,
    ControlStream
  };

  /** Enum for possible message types. */
  enum MessageType
  {
    MessageType_VIDEO,
    MessageType_CONTROL,
    MessageType_SYNC_SEND,
    MessageType_SYNC_RESPONSE
  };

  extern const unsigned short KILO_BYTE;
  extern const unsigned int MEGA_BYTE;
  extern const uint64_t GIGA_BYTE;

  /** Callback function definition used to notify when a new frame is received. */
  typedef boost::function<void (uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength)> OnReceiveFrameFunction;

  /**
   * Presentation layer used to interface communication between the user and the communication module.
   * Contains multiple framestreams which serve as data streams.
   * There usually is one framestream for the control channel and
   * one or more for the video channels.
   */
  class Connection
  {
    public:
      /**
       * Enum for possible stream control modes.
       * Static: the slave uses a constant compression value.
       * Local: the slave applies the local stream control algorithm.
       */
      enum StreamControlMode
      {
        StreamControlMode_STATIC,
        StreamControlMode_ACTIVE
      };

      /**
       * Constructor.
       * @param[in] syncCount Indicates the number of messages sent in the clock synchronization phase.
       * @param[in] syncPeriod Indicates the constant delay between the messages sent in the clock synchronization phase.
       */
      Connection(unsigned int syncCount, unsigned int syncPeriod);

      /** Destructor. */
      ~Connection();

      /** Used to calculate clock differences. Should be called before initiating data transfers. */
      void CalculateClockDifferences();

      /** Used to set the callback function to be called when the connection receives a new frame. */
      void SetReceiveFunction(OnReceiveFrameFunction onReceiveFrameFunction);

      /**
       * Used to add a new framestream representing a data flow.
       * @param[in] streamType Indicates the type of the stream.
       * @param[in] listenPort The number of the port where incoming frames will be received.
       * @param[in] remoteIp The ip of the remote host where outgoing frames will be sent.
       * @param[in] remotePort The port used for sending outgoing frames.
       * @param[in] chunkSize The size of a chunk. Frames will be split according to this.
       * @param[in] receiveBufferSize The size of the receive buffer of the socket.
       * @param[in] sendQueueLength The maximum number of frames queued for sending.
       * @param[in] controlTimerPeriod The period of the control timer.
       * @param[in] statisticsConfig Holds configuration data for the statistics.
       * @return
       */
      unsigned int AddFrameStream(
          StreamType streamType,
          unsigned short listenPort,
          const string& remoteIp,
          unsigned short remotePort,
          unsigned short chunkSize,
          unsigned int receiveBufferSize,
          unsigned int sendQueueLength,
          unsigned int controlTimerPeriod,
          boost::shared_ptr<Utils::StatisticsConfig> statisticsConfig
      );

      /**
       * Used to remove a frame stream.
       * @param[in] frameStreamID The ID of the framestream.
       */
      void RemoveFrameStream(uint32_t frameStreamID);

      /**
       * Called by the underlying framestream when a new frame is received.
       * @param[in] frameStreamID Number indicating the id of the caller framestream.
       * @param[in] messageType Indicates the type of the message received.
       * @param[in] data Array holding the payload.
       * @param[in] dataLength Number indicating the size of the payload.
       */
      void OnFrameRead(uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength);

      /**
       * Used to send a frame.
       * @param[in] frameStreamID Number indicating the id of the framestream to be used.
       * @param[in] messageType Indicates the type of the message.
       * @param[in] data Array holding the payload.
       * @param[in] dataLength Number indicating the size of the payload.
       */
      void WriteFrame(uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength);

      /**
       * Called by the Statistics through the frameStream after the window for received control messages is full.
       * @param[in] frameStreamId The ID of the framestream used for the control frames.
       * @param[in] receiveJitterVariance The variance of the receive jitter.
       * @param[in] averageDelay The average delay.
       * @param[in] lastReceiveTimestamp The last receive timestamp.
       */
      void CalculateControl(uint32_t frameStreamId, double receiveJitterVariance, double averageDelay, uint64_t lastReceiveTimestamp);

      /**
       * Used to query and reset the totalReceived and totalExpected values for a given framestream.
       * @param[in] frameStreamId The id of the framestream.
       * @param[in] totalReceived The total number of bytes received by the framestream (before reset).
       * @param[in] totalExpected The total number of bytes expected by the framestream (before reset).
       */
      void GetCounterData(uint32_t frameStreamId, uint32_t& totalReceived, uint32_t& totalExpected);

      /** Getter for the stream control mode. */
      StreamControlMode GetStreamControlMode() const;

      /**
       * Used to create the stream control.
       * @param[in] config The configuration data used to create the stream control.
       */
      void CreateStreamControl(boost::shared_ptr<Utils::StreamControlConfig> config);

      /** Used to get the local compression. */
      double GetLocalCompression() const;

    private:

      /** Used to log the newly created control.
       * @param[in] frameStreamId The id of the framestream for which the control was calculated (control stream).
       * @param[in] receiveJitterVariance The variance of the receive jitter.
       * @param[in] averageDelay The average delay
       * @param[in] lastReceiveTimestamp The last receive timestamp.
       */
      void OnStatisticsComputed(uint32_t frameStreamId, double receiveJitterVariance, double averageDelay, uint64_t lastReceiveTimestamp);

      /** The vector holding the frame streams. */
      vector<boost::shared_ptr<FrameStream> >* m_pFrameStreamVector;

      /** Callback function called when a frame is received. */
      OnReceiveFrameFunction m_OnReceiveFrameFunction;

      /** Indicates the number of messages sent in the clock synchronization phase. */
      unsigned int m_SyncCount;

      /** Indicates the constant delay between the messages sent in the clock synchronization phase. */
      unsigned int m_SyncPeriod;

      /** The stream control instance. */
      boost::shared_ptr<StreamControl> m_StreamControl;

      /** The stream control mode employed. */
      StreamControlMode m_StreamControlMode;

      /** Log file. */
      boost::shared_ptr<Utils::Log> m_Log;

      /** The id of the control stream. */
      uint32_t m_ControlStreamID;

      /** The id of the video stream. */
      uint32_t m_VideoStreamID;

      /** The local compression calculated by the stream control. */
      double m_LocalCompression;
  };

} // end namespace Communicaton

#endif // Connection_h
