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


#include "FrameStream.h"
#include "Core.h"
#include "DataGramPack.h"
#include "DataGram.h"
#include "Log.h"
#include "Statistics.h"
#include "PerformanceTimer.h"
#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <sstream>

using namespace boost;
using namespace Utils;

namespace Communication
{
  FrameStream::FrameStream(Connection* connection,
                           uint32_t frameStreamId,
                           StreamType streamType,
                           unsigned short listenPort,
                           const string& remoteIp,
                           unsigned short remotePort,
                           unsigned int receiveBufferSize,
                           unsigned int sendQueueLength,
                           unsigned short chunkSize,
                           unsigned int controlTimerPeriod,
                           boost::shared_ptr<Utils::StatisticsConfig> statisticsConfig) :
    m_Connection(connection),
    m_FrameStreamId(frameStreamId),
    m_StreamType(streamType),
    m_LocalEndPoint(udp::v4(), listenPort),
    m_RemoteEndPoint(asio::ip::address::from_string(remoteIp), remotePort),
    m_SendQueueLength(sendQueueLength),
    m_ChunkSize(chunkSize),
    m_NextFrameId(1),
    m_IsSending(false),
    m_ClockDifference(std::numeric_limits<double>::infinity()),
    min_Mr_Ms(0),
    m_LastId(0),
    m_totalReceived(0),
    m_totalExpected(0)
  {
    if (statisticsConfig.get())
      m_StatisticsPtr.reset(new Statistics(this, statisticsConfig, controlTimerPeriod));

    m_DataGramPtr.reset(new DataGram(*this, receiveBufferSize));
  }

  FrameStream::~FrameStream()
  {
    m_IsSending = false;
  }

  void FrameStream::SendSync() {
    int dataLength = 2 * sizeof(uint64_t);
    char* payload = new char[dataLength];
    boost::shared_array<char> data(payload);

    PushFrame(MessageType_SYNC_SEND, data, dataLength);
  }

  double FrameStream::GetClockDifference() const {
    return m_ClockDifference;
  }

  void FrameStream::SetClockDifference(double clockDifference) {
    m_ClockDifference = clockDifference;
  }

  const Connection* FrameStream::GetConnection() const {
    return m_Connection;
  }

  uint32_t FrameStream::getFrameStreamId() const
  {
    return m_FrameStreamId;
  }

  StreamType FrameStream::getStreamType() const
  {
    return m_StreamType;
  }

  unsigned short FrameStream::GetMaxDataSize()
  {
    return m_ChunkSize - sizeof(DataGramPackHeader);
  }

  void FrameStream::SetChunkSize(unsigned short chunkSize)
  {
    if (chunkSize > MAXIMUM_DATAGRAM_LENGTH)
    {
      m_ChunkSize = MAXIMUM_DATAGRAM_LENGTH;
    }
    else
    {
      if (chunkSize < sizeof(DataGramPackHeader))
      {
        // Data size = 0
        m_ChunkSize = sizeof(DataGramPackHeader);
      }
      else
      {
        m_ChunkSize = chunkSize;
      }
    }
  }

  unsigned short FrameStream::GetChunkSize()
  {
    return m_ChunkSize;
  }

  void FrameStream::WriteDatagram()
  {
    // Abort if sending is already in progress
    if (m_IsSending)
    {
      return;
    }

    SendDatagram();
  }

  void FrameStream::PushFrame(MessageType messageType, boost::shared_array<char> data, uint32_t dataLength)
  {
    // Add this list to the existing outgoing list
    m_OutDatagramListMutex.lock();

    // check if the maximum queue size if exceeded
    if (m_OutDatagramList.size() >= m_SendQueueLength) {
      // drop the chunks belonging to the oldest frame id in the queue
      uint64_t deleteId;
      list<boost::shared_ptr<DataGramPack> >::iterator it = m_OutDatagramList.begin();

      while (it != m_OutDatagramList.end()) {
        if ((*it)->header.sequenceNumber == 1) {
          deleteId = (*it)->header.id;
          break;
        }
        ++it;
      }

      while (it != m_OutDatagramList.end() && (*it)->header.id == deleteId) {
        m_OutDatagramList.erase(it++);
      }
    }

    // split and append data
    SplitInDatagrams(m_NextFrameId++, messageType, data, dataLength);

    m_OutDatagramListMutex.unlock();

    // Call write function because we have something to send
    WriteDatagram();
  }

  void FrameStream::SendDatagram()
  {
    m_OutDatagramListMutex.lock();

    if (m_OutDatagramList.size() == 0)
    {
      // Nothing to do
      m_IsSending = false;
      m_OutDatagramListMutex.unlock();
      return;
    }

    boost::shared_ptr<DataGramPack> dataGramPackPtr;

    // Get the first element from out list
    dataGramPackPtr = m_OutDatagramList.front();

    // Push timestamp
    uint64_t t;
    uint64_t currentTimeStamp;
    PerformanceTimer::GetTimeStamp(t);
    currentTimeStamp = t;

    dataGramPackPtr->header.sourceTimeStamp = currentTimeStamp;

    // Remove that element from list
    m_OutDatagramList.pop_front();

    // Send datagram pack
    m_DataGramPtr->AsyncSend(dataGramPackPtr);

    m_IsSending = !m_OutDatagramList.empty();

    m_OutDatagramListMutex.unlock();
  }

  void FrameStream::OnRead(boost::shared_ptr<DataGramPack> dataGramPackPtr)
  {
    int tolerance = 5;
    MessageType messageType = static_cast<MessageType>(dataGramPackPtr->header.messageType);

    if (messageType == MessageType_CONTROL)
    {
      // Statistics
      if (m_StatisticsPtr != 0)
        m_StatisticsPtr->PushElement(dataGramPackPtr);

      m_Connection->OnFrameRead(m_FrameStreamId, messageType, dataGramPackPtr->data, dataGramPackPtr->header.dataLength);
    }
    else if (messageType == MessageType_VIDEO)
    {
      // Statistics
      if (m_StatisticsPtr != 0)
        m_StatisticsPtr->PushElement(dataGramPackPtr);

      uint64_t newFrameId = dataGramPackPtr->header.id;

      m_videoStatisticsMutex.lock();
      m_totalReceived++;

      if (m_VideoChunkBuffer.size() > 0  && m_VideoChunkBuffer.find(newFrameId) == m_VideoChunkBuffer.end()) {
        // if a chunk belonging to a new frame has arrived
        int64_t idDiff = newFrameId - m_LastId;
        if (idDiff > 0) {
          uint32_t chunkSize = m_VideoChunkBuffer.rbegin()->second.rbegin()->second->header.sequenceCount;
          m_totalExpected += static_cast<uint32_t>(idDiff) * chunkSize;
        }
      }

      if (newFrameId > m_LastId)
        m_LastId = newFrameId;

      if (!m_VideoChunkBuffer.empty() && newFrameId > m_VideoChunkBuffer.begin()->first + tolerance)
      {
        // NEW message id. Drop all older then newID - tolerance
        map<uint64_t, map<uint32_t, boost::shared_ptr<DataGramPack> > >::iterator it;
        for (it = m_VideoChunkBuffer.begin(); it != m_VideoChunkBuffer.end(); )
          if (it->first < newFrameId - tolerance) {
            it->second.clear();
            m_VideoChunkBuffer.erase(it++);
          } else
            break;
      }

      // Add the received data packet
      m_VideoChunkBuffer[newFrameId].insert(make_pair(dataGramPackPtr->header.sequenceNumber, dataGramPackPtr));

      // Check if all packets belonging to this id have been received
      if (m_VideoChunkBuffer[newFrameId].begin()->second->header.sequenceCount == m_VideoChunkBuffer[newFrameId].size())
      {
        // Merge will notify users.
        MergeDataGrams(newFrameId);

        if (m_VideoChunkBuffer.size() == 1) {
          uint32_t chunkSize = m_VideoChunkBuffer[newFrameId].begin()->second->header.sequenceCount;
          m_totalExpected += chunkSize;
        }

        // Delete items.
        m_VideoChunkBuffer[newFrameId].clear();
        m_VideoChunkBuffer.erase(newFrameId);
      }

      m_videoStatisticsMutex.unlock();
    }
    else if (messageType == MessageType_SYNC_SEND) {
      // get the current time since epoch time
      uint64_t t;
      uint64_t currentTimeStamp;
      PerformanceTimer::GetTimeStamp(t);
      currentTimeStamp = t;

      // prepare response
      int dataLength = 2 * sizeof(uint64_t);
      char* payload = new char[dataLength];
      memcpy(payload, &dataGramPackPtr->header.sourceTimeStamp, sizeof(uint64_t)); // send time
      memcpy(payload + sizeof(uint64_t), &currentTimeStamp, sizeof(uint64_t)); // receive time
      boost::shared_array<char> data(payload);

      m_OutDatagramListMutex.lock();
      SplitInDatagrams(m_NextFrameId++, MessageType_SYNC_RESPONSE, data, dataLength);
      m_OutDatagramListMutex.unlock();

      // send response
      WriteDatagram();
    }
    else if (messageType == MessageType_SYNC_RESPONSE) {
      // get the current timestamp
      uint64_t t;
      uint64_t currentTimeStamp;
      PerformanceTimer::GetTimeStamp(t);
      currentTimeStamp = t;

      // get the original send timestamp
      uint64_t sendTimeStamp;
      memcpy(&sendTimeStamp, dataGramPackPtr->data.get(), sizeof(uint64_t));

      // get the time when the message was received on the other side
      uint64_t receiveTimeStamp;
      memcpy(&receiveTimeStamp, dataGramPackPtr->data.get() + sizeof(uint64_t), sizeof(uint64_t));

      // When the master starts the measurement
      int64_t Sr_Ms = receiveTimeStamp / 10 - sendTimeStamp / 10;
      int64_t Mr_Ms = currentTimeStamp / 10 - sendTimeStamp / 10;
      int64_t Ss_Sr = dataGramPackPtr->header.sourceTimeStamp / 10 - receiveTimeStamp / 10;

      double diff = (Sr_Ms - (Mr_Ms - Ss_Sr) / 2.0) / 100.0;
      clockDifferences.push_back(SyncData(receiveTimeStamp, sendTimeStamp, dataGramPackPtr->header.sourceTimeStamp, currentTimeStamp, diff));

      m_ClockDifferenceMutex.lock();

      assert(Mr_Ms >= 0);
      if (min_Mr_Ms == 0 || Mr_Ms < min_Mr_Ms) {
        m_ClockDifference = diff;
        min_Mr_Ms = Mr_Ms;
      }
      m_ClockDifferenceMutex.unlock();
    }
    else {
      m_Connection->OnFrameRead(m_FrameStreamId, messageType, dataGramPackPtr->data, dataGramPackPtr->header.dataLength);
    }
  }

  void FrameStream::SplitInDatagrams(
      uint64_t id,
      MessageType messageType,
      boost::shared_array<char> data,
      const unsigned int dataLength)
  {
    unsigned short maximumDataLength = GetMaxDataSize();
    if (maximumDataLength <= 0)
    {
      // should not reach this line
      return;
    }

    unsigned long count = dataLength / maximumDataLength;
    count += (dataLength % maximumDataLength > 0);

    unsigned long curPos = 0;
    for (unsigned long i = 0; i < count; i++)
    {
      char *dataChunk = new char[maximumDataLength];
      int dataChunkLength = std::min<int>(maximumDataLength, dataLength - curPos);
      memcpy(dataChunk, data.get() + curPos, dataChunkLength);

      boost::shared_ptr<DataGramPack> dataGramPackPtr = CreateDataGramPack(
                                                          id,
                                                          i + 1,
                                                          count,
                                                          messageType,
                                                          boost::shared_array<char>(dataChunk),
                                                          dataChunkLength);

      // Add to list
      m_OutDatagramList.push_back(dataGramPackPtr);

      curPos += maximumDataLength;
    }
  }

  void FrameStream::MergeDataGrams(uint64_t frameID)
  {
    // First we estimate the total length of the received frame.
    // Because the two maps are implicitly sorted based on ID and sequence number the total length
    // equals the maximum data size in one packet multiplied by the packet count (in this frame).
    uint32_t frameLength =  m_VideoChunkBuffer[frameID].begin()->second->header.dataLength * (m_VideoChunkBuffer[frameID].size() - 1);

    // The last packet might not have the full length (= max data size), this is added separately.
    frameLength += m_VideoChunkBuffer[frameID][m_VideoChunkBuffer[frameID].size()]->header.dataLength;

    // Reserve space for the frame data.
    boost::shared_array<char> frameData(new char[frameLength]);

    // Used to calculate the individual data parts total length.
    uint32_t partialLengths = 0;

    // Note: sequence numbers start from 1. (1of3, 2of3, 3of3)
    for (uint32_t sequenceNumberIndex = 1; sequenceNumberIndex <= m_VideoChunkBuffer[frameID].size(); sequenceNumberIndex++)
    {
      // Make sure the the data is copied at the end of the buffer.
      memcpy(frameData.get() + partialLengths,
             m_VideoChunkBuffer[frameID][sequenceNumberIndex]->data.get(),
             m_VideoChunkBuffer[frameID][sequenceNumberIndex]->header.dataLength);

      // Add the newly copied data part length
      partialLengths += m_VideoChunkBuffer[frameID][sequenceNumberIndex]->header.dataLength;
    }

    // The estimated and the calculated lengths must be equal!
    assert(frameLength == partialLengths);

    // Notify higher levels
    m_Connection->OnFrameRead(m_FrameStreamId, static_cast<MessageType>(m_VideoChunkBuffer[frameID][1]->header.messageType), frameData, frameLength);
  }

  void FrameStream::GetCounterData(uint32_t& totalReceived, uint32_t& totalExpected)
  {
    m_videoStatisticsMutex.lock();
    totalReceived = m_totalReceived;
    totalExpected = m_totalExpected;
    m_totalReceived = 0;
    m_totalExpected = 0;
    m_videoStatisticsMutex.unlock();
  }

  udp::endpoint& FrameStream::GetRemoteEndPoint()
  {
    return m_RemoteEndPoint;
  }

  udp::endpoint& FrameStream::GetLocalEndPoint()
  {
    return m_LocalEndPoint;
  }

  const string FrameStream::GetLocalPort() const {
    return lexical_cast<string>(m_LocalEndPoint.port());
  }

  void FrameStream::OnWrite()
  {
    SendDatagram();
  }

  void FrameStream::CalculateControl(double receiveJitterVariance, double averageDelay, uint64_t lastReceiveTimestamp)
  {
    m_Connection->CalculateControl(m_FrameStreamId, receiveJitterVariance, averageDelay, lastReceiveTimestamp);
  }

  void FrameStream::LogClockDifferences() const {
    if (m_StatisticsPtr != 0) {
        boost::shared_ptr<Log> log = m_StatisticsPtr->GetLog();

        stringstream ss;
        for (unsigned int i = 0; i < clockDifferences.size(); i++)
          ss << "Sync " << i + 1 << ": recv = " << clockDifferences[i].receive << " send = " << clockDifferences[i].send
             << " src = " << clockDifferences[i].source << " curr = " << clockDifferences[i].current
             << " diff = " << fixed << setw(10) << setprecision(2) << clockDifferences[i].diff << endl;
        log->Write(ss.str());
    }
  }

} // end namespace Communication
