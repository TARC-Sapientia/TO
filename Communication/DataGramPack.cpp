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


#include "DataGramPack.h"

namespace Communication
{

  boost::shared_ptr<DataGramPack> CreateDataGramPack(uint64_t id, uint32_t sequenceNumber, uint32_t sequenceCount, MessageType messageType, boost::shared_array<char> data, uint32_t dataLength)
  {
    DataGramPack * dp = new DataGramPack();

    dp->header.id = id;
    dp->header.sequenceNumber = sequenceNumber;
    dp->header.sequenceCount = sequenceCount;
    dp->header.sourceTimeStamp = 0;
    dp->header.messageType = static_cast<uint32_t>(messageType);
    dp->header.dataLength = dataLength;
    dp->data = data;

    return boost::shared_ptr<DataGramPack>(dp);
  }

  unsigned short CalculateDataGramPackSize(const boost::shared_ptr<DataGramPack> dataGramPackPtr)
  {
    return sizeof(DataGramPackHeader) + dataGramPackPtr->header.dataLength;
  }

} // end namespace Communication

