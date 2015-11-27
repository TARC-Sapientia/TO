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
#ifndef DataGramPack_h
#define DataGramPack_h

#include <string>
#include <boost/shared_array.hpp>
#include "Connection.h"

using namespace std;

namespace Communication
{

  /**
   * IPv4 and IPv6 define minimum reassembly buffer size,
   * the minimum datagram size that we are guaranteed any
   * implementation must support. For IPv4, this is 576 bytes.
   * IPv6 raises this to 1,500 bytes.
   *
   * The correct maximum UDP message size is 65507,
   * as determined by the following formula:
   * 0xffff - (sizeof(IP Header) + sizeof(UDP Header)) = 65535-(20+8) = 65507
   *
   * Bandwidth > ChunkSize
   */

  /** The maximum size of a datagram. */
  const unsigned short MAXIMUM_DATAGRAM_LENGTH = 63 * 1024;		// < 65507  .. max 63 KILO

  /**
   * Definition for the header part of a datagram.
   * Each frame may be split into multiple DatagramPacks.
   */
  struct DataGramPackHeader
  {
      /** Unique identifier for each DataGramPack. */
      uint64_t id;

      /** The number of the current chunk. Used to merge chunks. */
      uint32_t sequenceNumber;

      /** The total number of chunks that the frame was split into. */
      uint32_t sequenceCount;

      /** Timestamp indicating the time at which the datagrampack was sent. */
      uint64_t sourceTimeStamp;

      /** The type of the frame. */
      uint32_t messageType;

      /** The size of the payload. */
      uint32_t dataLength;
  };

  /**
   * The class the represents a datagram.
   */
  struct DataGramPack
  {
      /** The header of the datagram. */
      DataGramPackHeader header;

      /** The payload of the datagram. */
      boost::shared_array<char> data;
  };

  /**
   * Helper function used to create a datagram with the parameters provided.
   * @param[in] id The unique identifier of the datagram.
   * @param[in] sequenceNumber The number of the current chunk.
   * @param[in] sequenceCount The total number of chunks for this datagram.
   * @param[in] messageType The type of the datagram.
   * @param[in] data The payload of the datagram.
   * @param[in] dataLength The size of the payload.
   * @return The datagram created.
   */
  boost::shared_ptr<DataGramPack> CreateDataGramPack(uint64_t id, uint32_t sequenceNumber, uint32_t sequenceCount, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength);

  /**
   * Helper function used to calculate the size of a datagram.
   * @param[in] dataGramPackPtr The datagram in question.
   * @return The size of the datagram.
   */
  unsigned short CalculateDataGramPackSize(const boost::shared_ptr<DataGramPack> dataGramPackPtr);

} // end namespace Communication

#endif // DataGramPack_h
