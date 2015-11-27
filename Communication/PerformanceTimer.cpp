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


#include "PerformanceTimer.h"

using namespace Communication;

#ifdef _WIN32
	static LARGE_INTEGER FREQUENCY = {0};
    static const bool _initialized = QueryPerformanceFrequency(&FREQUENCY) ? true : false;
	static const double DECODE_RATIO = (double)1000000 / (double)FREQUENCY.QuadPart;
#endif

PerformanceTimer::PerformanceTimer()
{
}

bool PerformanceTimer::GetTimeStamp(TimeStamp& timeStamp)
{
	bool ret = false;

	#ifdef _WIN32
	QueryPerformanceCounter(&timeStamp);
	ret = true;
	#else
	ret = clock_gettime(CLOCK_MONOTONIC, &timeStamp	) == 0;
	#endif

	return ret;
}

bool PerformanceTimer::GetTimeStamp(uint64_t& timestamp)
{
	PerformanceTimer::TimeStamp ts;
    if (!GetTimeStamp(ts))
		return false;
	Decode(ts, timestamp);

	return true;
}

void PerformanceTimer::Decode(TimeStamp& timeStamp, uint64_t& ull)
{
	#ifdef _WIN32
		ull = (unsigned long long)(timeStamp.QuadPart * DECODE_RATIO);
	#else
		ull = (unsigned long long)timeStamp.tv_sec * 1000 * 1000 + (unsigned long long)timeStamp.tv_nsec / 1000;	
	#endif
}

bool operator == (PerformanceTimer::TimeStamp& t1, PerformanceTimer::TimeStamp& t2)
{
	#ifdef _WIN32
		return t1.QuadPart == t2.QuadPart;
	#else
		return t1.tv_sec == t2.tv_sec && (t1.tv_nsec / 1000) == (t2.tv_nsec / 1000);
	#endif
}
