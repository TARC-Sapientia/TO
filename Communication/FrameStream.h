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
#ifndef FrameStream_h
#define FrameStream_h

#include <string>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "Connection.h"

using namespace std;
using boost::asio::ip::udp;

namespace Communication
{
  struct DataGramPack;
  class DataGram;
  class Statistics;

  /**
   * The class that represents a data flow.
   * It is used to send and receive frames from a remote endpoint.
   * Before sending them, frames are split into chunks (datagrams).
   * Chunks received are automatically merged (if possible - otherwise
   * they are discarded) before being forwarded to the Connection.
   */
  class FrameStream
  {
    public:
      /**
       * Constructor.
       * @param[in] connection The connection that the stream belongs to.
       * @param[in] frameStreamId The unique identifier of the framestream.
       * @param[in] streamType The type of the stream.
       * @param[in] listenPort The port that the stream listens on.
       * @param[in] remoteIp The IP of the remote host.
       * @param[in] remotePort The port number for the remote connection.
       * @param[in] receiveBufferSize The size of the socket's receive buffer.
       * @param[in] sendQueueLength The maximum number of frames queued for sending.
       * @param[in] chunkSize The maximum size of a chunk. Frames are split accordingly.
       * @param[in] controlTimerPeriod The delay between control frame.
       * @param[in] statisticsConfig Holds configuration data for the statistics.
       */
      FrameStream(Connection* connection,
                  boost::uint32_t frameStreamId,
                  StreamType streamType,
                  unsigned short listenPort,
                  const string& remoteIp,
                  unsigned short remotePort,
                  unsigned int receiveBufferSize,
                  unsigned int sendQueueLength,
                  unsigned short chunkSize,
                  unsigned int controlTimerPeriod,
                  boost::shared_ptr<Utils::StatisticsConfig> statisticsConfig
                  );

      /** Destructor. */
      ~FrameStream();

      /**
       * Getter for the connection member.
       * @return The connection.
       */
      const Connection* GetConnection() const;

      /**
       * Getter for the clock difference.
       * @return the clock difference.
       */
      double GetClockDifference() const;

      /**
       * Setter for the clock difference.
       * @param[in] clockDifference The new value for the clock difference.
       */
      void SetClockDifference(double clockDifference);

      /**
       * Getter for the framestream's id.
       * @return The id of the framestream.
       */
      unsigned int getFrameStreamId() const;

      /**
       * Getter for the framestream's type.
       * @return The type of the framestream.
       */
      StreamType getStreamType() const;

      /**
       * Getter for the local endpoint.
       * @return The local IP.
       */
      udp::endpoint& GetLocalEndPoint();

      /**
       * Getter for the remote endpoint.
       * @return The remote IP.
       */
      udp::endpoint& GetRemoteEndPoint();

      /**
       * Getter for the local listening port.
       * @return The local port.
       */
      const string GetLocalPort() const;

      /**
       * Setter for the maximum size of a chunk.
       * @param[in] chunkSize The new value of the chunkSize.
       */
      void SetChunkSize(unsigned short chunkSize);

      /**
       * Getter for the maximum size of a chunk.
       * @return The maximum size of a chunk.
       */
      unsigned short GetChunkSize();

      /**
       * Used to calculate the maximum size of the payload with respect to the datagram header and chunk sizes.
       * @return The maximum size of the payload.
       */
      unsigned short GetMaxDataSize();

      /**
       * Used to query and reset the totalReceived and totalExpected values of the stream.
       * @param[in] totalReceived The total number of bytes received by the framestream (before reset).
       * @param[in] totalExpected The total number of bytes expected by the framestream (before reset).
       */
      void GetCounterData(uint32_t& totalReceived, uint32_t& totalExpected);

      /**
       * Entry point used to send frames.
       * @param[in] messageType The type of the frame.
       * @param[in] data The payload of the frame.
       * @param[in] dataLength The size of the payload.
       */
      void PushFrame(MessageType messageType, boost::shared_array<char> data, boost::uint32_t dataLength);

      /**
       * Used to send a frame used in the clock synchronization phase.
       */
      void SendSync();

      /**
       * Function called by the underlying layer when a datagram (chunk) is received.
       * The Statistics is notified about the incoming control or video message.
       * Control messages are forwarded immediately to the Connection.
       * Incoming video chunks are buffered until the whole datagram can be merged.
       * Once merged, the data is delivered to the Connection.
       * If a message with a new id is received, all buffered chunks having an id
       * outside of the tolerance range are dropped, thus avoiding buildups.
       * Synchronization messages used for clock difference calculus are handled internally.
       * @param[in] dataGramPackPtr The datagram received.
       */
      void OnRead(boost::shared_ptr<DataGramPack> dataGramPackPtr);

      /**
       * Function called when the underlying layer completes sending a datagram.
       * Tries to send the next datagram, if any.
       */
      void OnWrite();

      /**
       * Called by the Statistics after the window for received control messages is full.
       * @param[in] receiveJitterVariance The variance of the receive jitter.
       * @param[in] averageDelay The average delay.
       * @param[in] lastReceiveTimestamp The last receive timestamp.
       */
      void CalculateControl(double receiveJitterVariance, double averageDelay, boost::uint64_t lastReceiveTimestamp);

      /**
       * Used to log the results of the clock synchronization.
       */
      void LogClockDifferences() const;

    private:

      /**
       * Structure used to hold synchronization data for each pair of sync send and response message.
       */
      typedef struct SyncData {
          /** Timestamp indicating the time when the SEND message was received on the other side. */
          int64_t receive;

          /** Timestamp indicating the time when the SEND message was sent to the other side. */
          int64_t send;

          /** Timestamp indicating the time when the RESPONSE message was sent by the other side. */
          int64_t source;

          /** Timestamp indicating the time when the RESPONSE message was received from the other side. */
          int64_t current;

          /** The clock difference calculated. */
          double diff;

          /**
           * Constructor.
           * @param[in] _receive The receive timestamp.
           * @param[in] _send The send timestamp.
           * @param[in] _source The source timestamp.
           * @param[in] _current The current timestamp.
           * @param[in] _diff The clock difference.
           */
          SyncData(int64_t _receive, int64_t _send, int64_t _source, int64_t _current, double _diff) :
            receive(_receive), send(_send), source(_source), current(_current), diff(_diff)
          {

          }
      } SyncData;

      /**
       * Splits a frame in chunks (DataGramPack's) if needed, then appends them to the send queue.
       * @param[in] id The auto-incremented unique identifier of each frame.
       * @param[in] messageType The type of the frame.
       * @param[in] data The payload of the frame.
       * @param[in] dataLength The size of the payload.
       */
      void SplitInDatagrams(boost::uint64_t id,
                            MessageType messageType,
                            boost::shared_array<char> data,
                            boost::uint32_t dataLength);

      /**
       * Used to merge frames if all chunks belonging to the provided id were received.
       * @param[in] id The unique id of the frame to be merged.
       */
      void MergeDataGrams(uint64_t id);

      /**
       * Checks if sending is currently in progress. If not, it tries to send the next datagram.
       */
      void WriteDatagram();

      /**
       * Used to send the next datagram, if any.
       * Called by:
       *  - WriteDatagram: when no sending is currently taking place and a new frame was split and added to the queue
       *  - OnWrite: a sending operation just finished, proceed with the next datagram
       */
      void SendDatagram();

      /**
       * The maximum size of a chunk. Frames are split into datagrams (DataGramPack class) accordingly.
       */
      unsigned short m_ChunkSize;

      /** Remote endpoint (ip + port). */
      udp::endpoint m_RemoteEndPoint;

      /** Local listening endpoint (ip + port). */
      udp::endpoint m_LocalEndPoint;

      /** Pointer to the underlying datagram layer. */
      boost::shared_ptr<DataGram> m_DataGramPtr;

      /** Pointer to the connection that the stream belongs to. */
      Connection* m_Connection;

      /** Pointer to the statistics. */
      boost::shared_ptr<Statistics> m_StatisticsPtr;

      /**
       * Structure used to buffer chunks to be merged or dropped.
       * It's format is: map<id, map<seqNr, datagramPack> >
       * By default it is sorted by id and seqNr.
       */
      map<boost::uint64_t, map<boost::uint32_t, boost::shared_ptr<DataGramPack> > > m_VideoChunkBuffer;

      /** List used to queue outgoing datagrams before being sent. */
      list<boost::shared_ptr<DataGramPack> > m_OutDatagramList;

      /** The maximum size of the m_OutDatagramList. If exceeded, chunks belonging to the oldest frame id are dropped. */
      unsigned int m_SendQueueLength;

      /** Mutex used to guard access to m_OutDatagramList. */
      boost::recursive_mutex m_OutDatagramListMutex;

      /**
       * Flag indicating whether a datagram is currently being sent.
       * Only one datagram is sent at any given time.
       * Remains true between calls to SendDatagram as long as the m_OutDatagramList in not empty.
       */
      bool m_IsSending;

      /** Unique auto-incremented id used to identify frame streams. */
      boost::uint32_t m_FrameStreamId;

      /** The type of the stream. It indicates the type of messages it handles. */
      StreamType m_StreamType;

      /** The clock difference calculated for this frame stream in [ms]. */
      double m_ClockDifference;

      /** Round trip time for a synchronization attempt (SEND + RESPONSE). */
      int64_t min_Mr_Ms;

      /** Mutex used to guard access to the clock difference. */
      boost::recursive_mutex m_ClockDifferenceMutex;

      /**
       * The latest (highest number) frame id received by the stream.
       * Used to measure the total number of bytes expected.
       */
      boost::uint64_t m_LastId;

      /** Unique auto-incremented identifier for frames. */
      boost::uint64_t m_NextFrameId;

      /** Mutex used to guard access to m_totalReceived and m_totalExpected. */
      boost::mutex m_videoStatisticsMutex;

      /** Indicates the total number of bytes received. */
      boost::uint32_t m_totalReceived;

      /** Indicates the total number of bytes expected. */
      boost::uint32_t m_totalExpected;

      /** Used to store synchronization data for each pair of sync send and response message. */
      vector<SyncData> clockDifferences;
  };

} // end namespace Communication

#endif // FrameStream_h
