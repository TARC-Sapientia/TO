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
#ifndef PERFORMANCE_TIMER_H
#define PERFORMANCE_TIMER_H

#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif
#endif // _WIN32

#include <boost/asio.hpp> 

#ifdef _WIN32
    #include <windows.h>
#else
    #include <sys/time.h>
    #include <time.h>
#endif

namespace Communication
{
    /**
     * Performance Timer class used to count microseconds for intervals up to one hour.
     */
    class PerformanceTimer
    {
      public:

        #ifdef _WIN32
            /** The timestamp container in Windows. */
            typedef LARGE_INTEGER TimeStamp;
        #else
            /** The timestamp container in Linux. */
            typedef struct timespec TimeStamp;
        #endif

        /** Constructor. */
        PerformanceTimer();

        /**
         * Get the current timestamp in the native format.
         * @param[out] timeStamp the current timestamp.
         * @return true if successful, false otherwise.
         */
        static bool GetTimeStamp(TimeStamp& timeStamp);

        /**
         * Get the current timestamp in microseconds.
         * @param[out] timeStamp the current timestamp.
         * @return true if successful, false otherwise.
         */
        static bool GetTimeStamp(uint64_t& timeStamp);

        /**
         * Decode a timestamp as an unsigned long long.
         * The decoded value represents microseconds.
         */
        static void Decode(TimeStamp& timeStamp, uint64_t& ull);

    };

    /** Equality operator */
    bool operator == (PerformanceTimer::TimeStamp& t1, PerformanceTimer::TimeStamp& t2);

} // end namespace Communication

#endif // PERFORMANCE_TIMER_H
