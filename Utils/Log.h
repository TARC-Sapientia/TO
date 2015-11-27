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
#ifndef Log_h
#define Log_h

#include <fstream>
#include <string>

#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif
#endif // _WIN32

namespace Utils
{
  /** Class used for general file logging. */
  class Log
  {
    public:
      /**
       * Constructor.
       * @param path Path to the log file.
       * @param fileName The name of the log file.
	   * @param appendTimeStamp The current timestamp will be added to the file name. By default this option is turned on.
       */
      Log(const std::string& path, const std::string& fileName, bool appendTimeStamp = true);

      /** Destructor. */
      ~Log();

      /**
       * Append to the log file.
       * @param message The message to be written.
       */
      void Write(const std::string& message);

    private:

      /** The stream used to write to the log file. */
      std::ofstream logFile;
  };

} // end namespace Utils

#endif // Log_h
