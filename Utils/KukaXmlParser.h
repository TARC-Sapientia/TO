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


#ifndef KUKA_XML_PARSER_H
#define KUKA_XML_PARSER_H

#include "IKukaXmlParser.h"
#include <libxml/parser.h>

namespace Utils
{
  /**
   * The KUKA youBot configuration XML parser class.
   */
  class KukaXmlParser : public IKukaXmlParser
  {
    public:

      /**
       * Constructor.
       */
      KukaXmlParser();

      /**
       * Used to parse the KUKA youBot configuration.
       * @param fileName The file containing the slave configuration.
       * @return a structure containing the parsed configurations.
       */
      virtual boost::shared_ptr<KukaManagerConfig> ParseKukaConfig(std::string fileName);

    private:

      /**
       * Used to parse the joint constraints configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<JointConstraints> HandleJointConstraintsTag(xmlNodePtr node);

      /**
       * Used to parse the Denavit-Hartenberg parameter configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<DenavitHartenbergParameters> HandleDenavitHartenbergParametersTag(xmlNodePtr node);

      /**
       * Used to parse the arm-home-position predefined joint configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<ArmJointHomePosition> HandleArmJointHomePositionTag(xmlNodePtr node);

      /**
       * Used to parse the arm-front-position predefined joint configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<ArmJointFrontPosition> HandleArmJointFrontPositionTag(xmlNodePtr node);

      /**
       * Used to parse the direct geometry configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<DirectGeometryJointZeroOffsets> HandleDirectGeometryJointZeroOffsetsTag(xmlNodePtr node);

      /**
       * Used to parse the gravity compensation dynamic parameters configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<GravityCompensationDynamicParameters> HandleGravityCompensationDynamicParametersTag(xmlNodePtr node);

      /**
       * Used to parse the gravity compensation joint zero offsets configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<GravityCompensationJointZeroOffsets> HandleGravityCompensationJointZeroOffsetsTag(xmlNodePtr node);

      /**
       * Used to parse KUKA youBot's extreme values configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<ExtremeKukaValuesConfig> HandleExtremeKukaValuesTag(xmlNodePtr node);

      /**
       * Used to parse the controller configuration tag.
       * @param node The XML node containing the configuration.
       * @return A pointer to the structure containing the parsed configurations.
       */
      boost::shared_ptr<ControllerParametersConfig> HandleControllerParametersTag(xmlNodePtr node);
  };

} // end namespace Utils

#endif // KUKA_XML_PARSER_H
