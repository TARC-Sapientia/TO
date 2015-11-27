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


#include "Statistics.h"
#include "Connection.h"
#include "FrameStream.h"
#include "PerformanceTimer.h"
#include "Core.h"
#include "CommunicationData.h"
#include <string>

using namespace Utils;

namespace Communication
{
Statistics::Statistics(
        FrameStream* frameStream,
        boost::shared_ptr<Utils::StatisticsConfig> statisticsConfig,
        unsigned int controlTimerPeriod) :
    m_FrameStream(frameStream),
    m_ControlTimerPeriod(controlTimerPeriod),
    window(statisticsConfig->window),
    alpha(statisticsConfig->alpha),
    averageDelayF(100)
{
    if (statisticsConfig->logPath != "")
        m_Log.reset(new Log(statisticsConfig->logPath, statisticsConfig->logName + m_FrameStream->GetLocalPort()));

    r_jitter = 0;
    r_jitterF = 0;
}

Statistics::~Statistics()
{

}

boost::shared_ptr<Log> Statistics::GetLog()
{
    return m_Log;
}

void Statistics::Reset()
{
    m_StatisticsDataVectorMutex.lock();

    m_StatisticsDataVector.clear();

    m_StatisticsDataVectorMutex.unlock();
}

void Statistics::PushElement(boost::shared_ptr<DataGramPack> dataGramPack)
{
    uint64_t t;
    uint64_t receiveTimeStamp;

    PerformanceTimer::GetTimeStamp(t);
    receiveTimeStamp = t;

    m_StatisticsDataVectorMutex.lock();

    unsigned int size = m_StatisticsDataVector.size();

    if (size > 0) {
        if (dataGramPack->header.id > m_StatisticsDataVector[size - 1].dataGramPackPtr->header.id) {
            r_jitter = ((double)receiveTimeStamp - m_StatisticsDataVector[size - 1].receiveTimeStamp);
            r_jitter /= 1000;
            r_jitter -= m_ControlTimerPeriod;
            r_jitterF = alpha * r_jitterF + (1 - alpha) * r_jitter;
        }
    }

    m_StatisticsDataVector.push_back(StatisticsData(dataGramPack, receiveTimeStamp, r_jitter, r_jitterF, m_FrameStream));
    if (size + 1 >= window)
        Core::get().getIoService().post(boost::bind(&Statistics::ComputeStatistics, this));

    m_StatisticsDataVectorMutex.unlock();
}

void Statistics::ComputeStatistics()
{
    m_StatisticsDataVectorMutex.lock();

    if (m_StatisticsDataVector.size() == 0)
    {
        m_StatisticsDataVectorMutex.unlock();
        return;
    }

    double m_rj_for_v_rj = 0;
    double receiveJitterVariance = 0;

    double m_rj = 0;
    double r_j = m_StatisticsDataVector[m_StatisticsDataVector.size() - 1].r_jitterF; // current value of r_j
    uint64_t lastTimestamp = m_StatisticsDataVector[m_StatisticsDataVector.size() - 1].receiveTimeStamp;

    if (m_FrameStream->getStreamType() == ControlStream) {
        for (unsigned int i = 0; i < m_StatisticsDataVector.size(); ++i) {
            receiveJitterVariance += abs(m_StatisticsDataVector[i].r_jitterF);
        }

        if (m_StatisticsDataVector.size() <= 1)
            receiveJitterVariance = 0;
        else
            receiveJitterVariance /= m_StatisticsDataVector.size() - 1;
    }

    uint64_t totalBytesReceived = 0;
    double averageDelay = 0;
    stringstream packetsStringStream;
    for (unsigned int i = 0; i < m_StatisticsDataVector.size(); ++i) {
        // calculate the sum of the delays
        averageDelay += m_StatisticsDataVector[i].delay;
        packetsStringStream << StatisticsDataToString(m_StatisticsDataVector[i]) << endl;
        totalBytesReceived += CalculateDataGramPackSize(m_StatisticsDataVector[i].dataGramPackPtr);
    }
    averageDelay /= m_StatisticsDataVector.size();

    m_StatisticsDataVector.clear();

    m_StatisticsDataVectorMutex.unlock();

    averageDelayF = alpha * averageDelayF + (1 - alpha) * averageDelay;

    if (m_FrameStream->getStreamType() == ControlStream)
        m_FrameStream->CalculateControl(receiveJitterVariance, averageDelayF, lastTimestamp);

    m_Log->Write(packetsStringStream.str());
}

string Statistics::StatisticsDataToString(const StatisticsData& statisticsData)
{
    stringstream ss;
    ss << fixed;
    ss << "ID: " << setw(6) << statisticsData.dataGramPackPtr->header.id << '\t';
    ss << "count: " << setw(4) << statisticsData.dataGramPackPtr->header.sequenceCount << '\t';
    ss << "nr: " << setw(4) << statisticsData.dataGramPackPtr->header.sequenceNumber << '\t';
    ss << "STS: " << setw(14) << statisticsData.dataGramPackPtr->header.sourceTimeStamp << '\t';
    ss << "RTS: " << setw(14) << statisticsData.receiveTimeStamp << '\t';
    ss << "length: " << setw(8) << statisticsData.dataGramPackPtr->header.dataLength << '\t';
    ss << "r_jitter: " << setw(8) << setprecision(3) << statisticsData.r_jitter << '\t';
    ss << "r_jitterF: " << setw(8) << setprecision(3) << statisticsData.r_jitterF << '\t';
    ss << "delay: " << setw(8) << setprecision(3) << statisticsData.delay << '\t';

    return ss.str();
}

} // end namespace Communication
