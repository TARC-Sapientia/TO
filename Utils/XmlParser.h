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
#ifndef XML_PARSER_H
#define XML_PARSER_H

#include "IXmlParser.h"
#include <libxml/parser.h>

namespace Utils
{
  /**
   * The master and slave configuration XML parser class.
   */
  class XmlParser : public IXmlParser
  {
    public:

      /**
       * Constructor.
       */
      XmlParser();

      /**
       * Used to parse the slave configuration.
       * @param fileName The file containing the slave configuration.
       * @return a structure containing the parsed configurations.
       */
      virtual boost::shared_ptr<SlaveConfig> ParseSlaveConfig(std::string fileName);

      /**
       * Used to parse the master configuration.
       * @param fileName The file containing the master configuration.
       * @return a structure containing the parsed configurations.
       */
      virtual boost::shared_ptr<MasterConfig> ParseMasterConfig(std::string fileName);

    private:

      /**
       * Used to parse the stream configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<StreamConfig> HandleStreamTag(xmlNodePtr node);

      /**
       * Used to parse the statistics configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<StatisticsConfig> HandleStatisticsTag(xmlNodePtr node);

      /**
       * Used to parse the stream control configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<StreamControlConfig> HandleStreamControlTag(xmlNodePtr node);

      /**
       * Used to parse the video capture configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<VideoCaptureConfig> HandleVideoCaptureTag(xmlNodePtr node);

      /**
       * Used to parse the remote slave configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<RemoteSlaveConfig> HandleRemoteSlaveTag(xmlNodePtr node);

      /**
       * Used to parse the haptic configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<HapticConfig> HandleHapticTag(xmlNodePtr node);

      /**
       * Used to parse the KUKA youBot configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<KukaConfig> HandleKukaTag(xmlNodePtr node);
  };

} // end namespace Utils

#endif // XML_PARSER_H
