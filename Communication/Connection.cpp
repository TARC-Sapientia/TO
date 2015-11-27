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


#include <iostream>
#include <boost/lexical_cast.hpp>

#include "Connection.h"
#include "Core.h"
#include "FrameStream.h"

using namespace std;

namespace Communication
{
  const unsigned short KILO_BYTE = 1024;
  const unsigned int MEGA_BYTE = 1024 * 1024;
  const uint64_t GIGA_BYTE = 1024 * 1024 * 1024;

  Connection::Connection(unsigned int syncCount, unsigned int syncPeriod) :
    m_SyncCount(syncCount),
    m_SyncPeriod(syncPeriod),
    m_ControlStreamID(0)
  {
    m_pFrameStreamVector = new vector<boost::shared_ptr<FrameStream> >();
  }


  Connection::~Connection()
  {
    m_pFrameStreamVector->clear();
    delete m_pFrameStreamVector;
  }


  void Connection::CalculateClockDifferences()
  {
    for (unsigned int i = 0; i < m_SyncCount; i++) {
      for (uint32_t frameStreamId = 0; frameStreamId < m_pFrameStreamVector->size(); ++frameStreamId)
        m_pFrameStreamVector->at(frameStreamId)->SendSync();

      boost::this_thread::sleep(boost::posix_time::millisec(m_SyncPeriod));
    }

    boost::this_thread::sleep(boost::posix_time::millisec(10 * m_SyncPeriod));

    // print results
    for (uint32_t frameStreamId = 0; frameStreamId < m_pFrameStreamVector->size(); ++frameStreamId) {
      cout << "FrameStream " << frameStreamId << " - clock difference: " << fixed << setw(10) << setprecision(2) << m_pFrameStreamVector->at(frameStreamId)->GetClockDifference() << " ms" << endl;
      m_pFrameStreamVector->at(frameStreamId)->LogClockDifferences();
    }
    cout << endl;
  }


  void Connection::SetReceiveFunction(OnReceiveFrameFunction onReceiveFrameFunction)
  {
    m_OnReceiveFrameFunction = onReceiveFrameFunction;
  }


  unsigned int Connection::AddFrameStream(
      StreamType streamType,
      unsigned short listenPort,
      const string& remoteIp,
      unsigned short remotePort,
      unsigned short chunkSize,
      unsigned int receiveBufferSize,
      unsigned int sendQueueLength,
      unsigned int controlTimerPeriod,
      boost::shared_ptr<Utils::StatisticsConfig> statisticsConfig)
  {
    uint32_t frameStreamId = m_pFrameStreamVector->size();

    m_pFrameStreamVector->push_back(boost::shared_ptr<FrameStream>(new FrameStream(
                                                                     this,
                                                                     frameStreamId,
                                                                     streamType,
                                                                     listenPort,
                                                                     remoteIp,
                                                                     remotePort,
                                                                     receiveBufferSize,
                                                                     sendQueueLength,
                                                                     chunkSize,
                                                                     controlTimerPeriod,
                                                                     statisticsConfig))
                                    );

    if (streamType == ControlStream)
      m_ControlStreamID = frameStreamId;
    else if (streamType == VideoStream)
      m_VideoStreamID = frameStreamId;

    return frameStreamId;
  }


  void Connection::RemoveFrameStream(uint32_t frameStreamID)
  {
    if (frameStreamID >= m_pFrameStreamVector->size())
    {
      return;
    }

    m_pFrameStreamVector->erase(m_pFrameStreamVector->begin() + frameStreamID);
  }


  void Connection::WriteFrame(uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength)
  {
    if (frameStreamID >= m_pFrameStreamVector->size())
    {
      return;
    }

    // fill
    m_pFrameStreamVector->at(frameStreamID)->PushFrame(messageType, data, dataLength);
  }


  void Connection::OnFrameRead(uint32_t frameStreamID, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength)
  {
    m_OnReceiveFrameFunction(frameStreamID, messageType, data, dataLength);
  }


  void Connection::CalculateControl(uint32_t frameStreamId, double receiveJitterVariance, double averageDelay, uint64_t lastReceiveTimestamp)
  {
    if (m_StreamControl)
      OnStatisticsComputed(frameStreamId, receiveJitterVariance, averageDelay, lastReceiveTimestamp);
  }


  void Connection::GetCounterData(uint32_t frameStreamId, uint32_t& totalReceived, uint32_t& totalExpected) {
    vector<boost::shared_ptr<FrameStream> >::iterator it;
    for (it = m_pFrameStreamVector->begin(); it != m_pFrameStreamVector->end(); it++)
      if ((*it)->getFrameStreamId() == frameStreamId)
        return (*it)->GetCounterData(totalReceived, totalExpected);

    return;
  }


  Connection::StreamControlMode Connection::GetStreamControlMode() const
  {
    return m_StreamControlMode;
  }


  void Connection::CreateStreamControl(boost::shared_ptr<Utils::StreamControlConfig> config) {
    if (!strcmp(config->mode.c_str(), "static")) {
      assert(config->parameters.find("u") != config->parameters.end());
      m_LocalCompression = boost::lexical_cast<double>(config->parameters["u"]);

      m_StreamControlMode = StreamControlMode_STATIC;
    }
    else if (!strcmp(config->mode.c_str(), "active")) {
      assert(config->parameters.find("u-max") != config->parameters.end());
      assert(config->parameters.find("u-min") != config->parameters.end());
      assert(config->parameters.find("u-inc") != config->parameters.end());
      assert(config->parameters.find("k-i") != config->parameters.end());
      assert(config->parameters.find("delta") != config->parameters.end());
      assert(config->parameters.find("mu") != config->parameters.end());
      assert(config->parameters.find("lambda") != config->parameters.end());

      m_StreamControl.reset(new StreamControl(
                              boost::lexical_cast<double>(config->parameters["u-max"]),
                              boost::lexical_cast<double>(config->parameters["u-min"]),
                              boost::lexical_cast<double>(config->parameters["u-inc"]),
                              boost::lexical_cast<double>(config->parameters["k-i"]),
                              boost::lexical_cast<double>(config->parameters["delta"]),
                              boost::lexical_cast<double>(config->parameters["mu"]),
                              boost::lexical_cast<double>(config->parameters["lambda"])));
      m_LocalCompression = boost::lexical_cast<double>(config->parameters["u-max"]);
      m_StreamControlMode = StreamControlMode_ACTIVE;
    }
    m_Log.reset(new Utils::Log(config->logPath, config->logName));
  }


  double Connection::GetLocalCompression() const
  {
    return m_LocalCompression;
  }


  void Connection::OnStatisticsComputed(uint32_t frameStreamID, double receiveJitterVariance, double averageDelay, uint64_t lastReceiveTimestamp) {
    if (frameStreamID == m_ControlStreamID) {
      double compression, e = 0;
      string mode;
      if (m_StreamControlMode == Connection::StreamControlMode_STATIC) {
        mode = "static";
      } else if (m_StreamControlMode == Connection::StreamControlMode_ACTIVE) {
        mode = "active";
        m_StreamControl->CalculateControl(receiveJitterVariance, averageDelay, e, m_LocalCompression);
      }

      compression = m_LocalCompression;

      // Print & log the newly calculated control
      stringstream ss;
      ss << fixed << setprecision(4)
         << "delay: " << setw(8) << averageDelay
         << " variance: " << setw(8) << receiveJitterVariance
         << " e:" << setw(8) << e
         << " u:" << setw(8) << compression
         << " time:" << setw(12) << lastReceiveTimestamp
         << " mode: " << setw(8) << mode
         << endl;

      m_Log->Write(ss.str());
    }
  }

} // end namespace Communication

