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
#ifndef CommunicationData_h
#define CommunicationData_h

namespace Communication
{
  /** Value indicating the number of columns in control data arrays. */
  const unsigned int controlDataCols = 9;

  /**
   * Structure holding control data.
   */
  struct ControlData
  {
      /** Array holding the position values. */
      double position[controlDataCols];

      /** Array holding the velocity values. */
      double velocity[controlDataCols];

      /** Array holding the force values. */
      double force[controlDataCols];

      /** Array holding the energies values. */
      double energy[controlDataCols];
  };
} // end namespace

#endif // CommunicationData_h 
