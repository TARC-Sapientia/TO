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


#ifndef HAPTIC_XML_PARSER_H
#define HAPTIC_XML_PARSER_H

#include "IHapticXmlParser.h"
#include <libxml/parser.h>

namespace Utils
{	
    /**
     * The haptic configuration XML parser class.
     */
    class HapticXmlParser : public IHapticXmlParser
    {

    public:

        /**
        * Constructor.
        */
        HapticXmlParser();

        /**
        * Used to parse the haptic configuration.
        * @param fileName The file containing the haptic configuration.
        * @return a structure containing the parsed configurations.
        */
        virtual boost::shared_ptr<HapticManagerConfig> ParseHapticManagerConfig(std::string fileName);

    private:

        /**
        * Used to parse the device configuration tag.
        * @param node The XML node containing the configuration.
        * @return A pointer to the structure containing the parsed configurations.
        */
        boost::shared_ptr<DeviceConfig> HandleDeviceTag(xmlNodePtr node);

        /**
        * Used to parse the constraints configuration tag.
        * @param node The XML node containing the configuration.
        * @return A pointer to the structure containing the parsed configurations.
        */
        boost::shared_ptr<ConstraintsConfig> HandleConstraintsTag(xmlNodePtr node);

        /**
        * Used to parse the threshold configuration tag.
        * @param node The XML node containing the configuration.
        * @return A pointer to the structure containing the parsed configurations.
        */
        boost::shared_ptr<ThresholdConfigForceFeedback> HandleThresholdConfigForceFeedbackTag(xmlNodePtr node);

        /**
        * Used to parse the log configuration tag.
        * @param node The XML node containing the configuration.
        * @return A pointer to the structure containing the parsed configurations.
        */
        boost::shared_ptr<LogConfig> HandleLogTag(xmlNodePtr node);

    };

} // end namespace Utils

#endif // HAPTIC_XML_PARSER_H
