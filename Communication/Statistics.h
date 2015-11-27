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
#ifndef STATISTICS_H
#define STATISTICS_H

#include <vector>
#include <fstream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "FrameStream.h"
#include "DataGramPack.h"
#include "Log.h"
using namespace std;
using namespace Utils;

namespace Communication
{
struct DataGramPack;

/**
 * Implementation of the network statistics which is used for stream control.
 * This class calculates the receive jitter and delivery delay for all the pushed frames,
 * and computes statistics on a predefined window of these information.
 */
class Statistics
{
public:

    /**
     * Constructor.
     * @param frameStream Pointer to the FrameStream whose data is analyzed.
     * @param statisticsConfig The configuration information.
     * @param controlTimerPeriod The sending period of the control data (used at jitter calculation).
     */
    Statistics(
            FrameStream* frameStream,
            boost::shared_ptr<Utils::StatisticsConfig> statisticsConfig,
            unsigned int controlTimerPeriod
            );

    /**
     * Resets the statistics.
     */
    void Reset();

    /**
     * Pushes a new packet for statistical analysis.
     * @param dataGramPack Pointer of the packet.
     */
    void PushElement(boost::shared_ptr<DataGramPack> dataGramPack);

    /**
     * Destructor.
     */
    ~Statistics();

    /**
     * Returns the logger object.
     * @return A pointer to the logger object.
     */
    boost::shared_ptr<Log> GetLog();

private:

    /** Holds the period of the control packet sending */
    const unsigned int m_ControlTimerPeriod;

    /**
     * Encapsulation of the statistic data.
     */
    typedef struct StatisticsData {
        /** Pointer to the analyzed data packet. */
        boost::shared_ptr<DataGramPack> dataGramPackPtr;

        /** The receive time. */
        uint64_t receiveTimeStamp;

        /** Holds the calculated receive jitter. */
        double r_jitter;

        /** Holds the filtered receive jitter. */
        double r_jitterF;

        /** Holds the calculated transmission delay */
        double delay; // [ms]

        /**
         * Constructor.
         * @param dataGramPackPtr Pointer to the analyzed data packet.
         * @param receiveTimeStamp The receive time.
         * @param r_jitter The receive jitter.
         * @param r_jitterF The filtered receive jitter.
         * @param frameStream Pointer to the frame stream.
         */
        StatisticsData(boost::shared_ptr<DataGramPack> dataGramPackPtr, uint64_t receiveTimeStamp, double r_jitter, double r_jitterF, FrameStream* frameStream) :
            dataGramPackPtr(dataGramPackPtr),
            receiveTimeStamp(receiveTimeStamp),
            r_jitter(r_jitter),
            r_jitterF(r_jitterF)
        {
            double sourceTime = dataGramPackPtr->header.sourceTimeStamp / 1000.0;
            double receiveTime = receiveTimeStamp / 1000.0;
            if (receiveTime + frameStream->GetClockDifference() < sourceTime) {
                frameStream->SetClockDifference(sourceTime - receiveTime);
                delay = 0;
            }
            else {
                delay = receiveTime - sourceTime + frameStream->GetClockDifference();
            }
        }
    } StatisticsData;

    /**
     * Calculates the statistics after the defined window of statistics data is cumulated.
     */
    void ComputeStatistics();

    /**
     * Formats the statistics data.
     * @param statisticsData The reference of the statistics data.
     * @return The formatted string.
     */
    string StatisticsDataToString(const StatisticsData& statisticsData);

    /** Vector to hold the statistics samples while the statistics window is cumulated. */
    vector<StatisticsData> m_StatisticsDataVector;

    /** Mutex to synchronize the access of the statistics vector. */
    boost::recursive_mutex m_StatisticsDataVectorMutex;

    /** Pointer to the frame stream providing the statistical data. */
    FrameStream* m_FrameStream;

    /** Pointer to the logging object */
    boost::shared_ptr<Log> m_Log;

    /** The size of the statistics window. */
    unsigned int window;

    /** The coefficient used at the jitter filtering. */
    double alpha;

    /** The actual receive jitter. */
    double r_jitter;

    /** The actual filtered receive jitter. */
    double r_jitterF;

    /** The actual filtered average delay. */
    double averageDelayF;
};
} // end namespace Communication

#endif // STATISTICS_H
