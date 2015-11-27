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
#ifndef StreamControl_h
#define StreamControl_h

#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <fstream>
#include <vector>

#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif
#endif // _WIN32

namespace Communication
{
/**
 * The implementation of the stream control algorithm.
 * This algorithm follows these steps:
 * 1. calculates a control error (e)
 * 2. compares the control error to a predefined threshold (delta)
 * 3. applies the control law to calculate the control signal (u)
 * For more details see the related scientific publications.
 */
class StreamControl
{
public:
    /**
     * Constructor.
     * @param[in] uMax Maximum value for the control signal.
     * @param[in] uMin Minimum value for the control signal.
     * @param[in] uInc The increment for the control signal.
     * @param[in] Ki Proportional coefficient used in the decreasing branch of the control law.
     * @param[in] delta The control threshold.
     * @param[in] mu The weight of the receive jitter variance used in the control error calculus.
     * @param[in] lambda The weight of the average delay used in the control error calculus.
     */
    StreamControl(
            double uMax,
            double uMin,
            double uInc,
            double Ki,
            double delta,
            double mu,
            double lambda);

    /**
     * Calculates the next value for the control signal.
     * @param[in] receiveJitterVariance The current value of the receive jitter variance.
     * @param[in] averageDelay The current value of the average delay.
     * @param[out] e The calculated control error.
     * @param[out] u The calculated control signal.
     */
    void CalculateControl(double receiveJitterVariance, double averageDelay, double& e, double& u);

    /**
     * Getter for the maximum value of the control signal.
     * @return The maximum value of the control signal.
     */
    double GetUMax() const;

    /**
     * Getter for the minimum value of the control signal.
     * @return The minimum value of the control signal.
     */
    double GetUMin() const;

private:
    /** The actual value of the control signal. */
    double u;

    /** The last calculated control error. */
    double last_e;

    /** The last calculated control signal. */
    double last_u;

    /** The maximum value for the control signal. */
    double uMax;

    /** The minimum value for the control signal. */
    double uMin;

    /** The increment for the control signal. */
    double uInc;

    /** Proportional coefficient used in the decreasing branch of the control law. */
    double Ki;

    /** The control threshold. */
    double delta;

    /** The weight of the receive jitter variance used in the control error calculus. */
    double mu;

    /** The weight of the average delay used in the control error calculus. */
    double lambda;
};

inline double StreamControl::GetUMax() const {
    return uMax;
}

inline double StreamControl::GetUMin() const {
    return uMin;
}

} // end namespace Communication

#endif // StreamControl_h
