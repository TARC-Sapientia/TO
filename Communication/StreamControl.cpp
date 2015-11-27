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
#include <iomanip>
#include "StreamControl.h"

using namespace std;
using namespace boost;

namespace Communication
{
StreamControl::StreamControl(double _uMax, double _uMin, double _uInc, double _Ki, double _delta, double _mu, double _lambda)
    : uMax(_uMax), uMin(_uMin), uInc(_uInc), Ki(_Ki), delta(_delta), mu(_mu), lambda(_lambda), u(uMax)
{
    last_e = 0;
    last_u = uMax;
}


void StreamControl::CalculateControl(double receiveJitterVariance, double averageDelay, double& _e, double& _u)
{
    _e = mu * receiveJitterVariance + lambda * averageDelay;

    if (_e < delta) {
        u = last_u + uInc;
        if (u > uMax)
            u = uMax;
        else
            last_u = u;
    }
    else {
        u = last_u - Ki * (_e + last_e) / 2;
        last_e = _e;
        if (u < uMin)
            u = uMin;
        last_u = u;
    }
    _u = u;
}

} // end Communication namespace
