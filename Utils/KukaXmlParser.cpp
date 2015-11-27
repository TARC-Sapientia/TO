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


#include "KukaXmlParser.h"
#include <boost/lexical_cast.hpp>
#include <string>
#include <iostream>
using namespace std;

namespace Utils
{
  KukaXmlParser::KukaXmlParser() { }

  IKukaXmlParser* getKukaXmlParser()
  {
    return new KukaXmlParser();
  }

  boost::shared_ptr<JointConstraints> KukaXmlParser::HandleJointConstraintsTag(xmlNodePtr node)
  {
    boost::shared_ptr<JointConstraints> jointConstraints(new JointConstraints());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"joint1-min");
    jointConstraints->minJoint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint1-max");
    jointConstraints->maxJoint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint2-min");
    jointConstraints->minJoint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint2-max");
    jointConstraints->maxJoint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint3-min");
    jointConstraints->minJoint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint3-max");
    jointConstraints->maxJoint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint4-min");
    jointConstraints->minJoint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint4-max");
    jointConstraints->maxJoint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint5-min");
    jointConstraints->minJoint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint5-max");
    jointConstraints->maxJoint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return jointConstraints;
  }

  boost::shared_ptr<DenavitHartenbergParameters> KukaXmlParser::HandleDenavitHartenbergParametersTag(xmlNodePtr node)
  {
    boost::shared_ptr<DenavitHartenbergParameters> denavitHartenbergParameters(new DenavitHartenbergParameters());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"joint1_d");
    denavitHartenbergParameters->dJoint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint1_a");
    denavitHartenbergParameters->aJoint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint1_alpha");
    denavitHartenbergParameters->alphaJoint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint2_d");
    denavitHartenbergParameters->dJoint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint2_a");
    denavitHartenbergParameters->aJoint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint2_alpha");
    denavitHartenbergParameters->alphaJoint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint3_d");
    denavitHartenbergParameters->dJoint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint3_a");
    denavitHartenbergParameters->aJoint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint3_alpha");
    denavitHartenbergParameters->alphaJoint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint4_d");
    denavitHartenbergParameters->dJoint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint4_a");
    denavitHartenbergParameters->aJoint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint4_alpha");
    denavitHartenbergParameters->alphaJoint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint5_d");
    denavitHartenbergParameters->dJoint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint5_a");
    denavitHartenbergParameters->aJoint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
    tmp = xmlGetProp(node, (const xmlChar *)"joint5_alpha");
    denavitHartenbergParameters->alphaJoint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return denavitHartenbergParameters;
  }

  boost::shared_ptr<ArmJointHomePosition> KukaXmlParser::HandleArmJointHomePositionTag(xmlNodePtr node)
  {
    boost::shared_ptr<ArmJointHomePosition> armJointHomePosition(new ArmJointHomePosition());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"joint1");
    armJointHomePosition->joint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint2");
    armJointHomePosition->joint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint3");
    armJointHomePosition->joint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint4");
    armJointHomePosition->joint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint5");
    armJointHomePosition->joint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return armJointHomePosition;
  }

  boost::shared_ptr<ArmJointFrontPosition> KukaXmlParser::HandleArmJointFrontPositionTag(xmlNodePtr node)
  {
    boost::shared_ptr<ArmJointFrontPosition> armJointFrontPosition(new ArmJointFrontPosition());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"joint1");
    armJointFrontPosition->joint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint2");
    armJointFrontPosition->joint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint3");
    armJointFrontPosition->joint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint4");
    armJointFrontPosition->joint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint5");
    armJointFrontPosition->joint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return armJointFrontPosition;
  }

  boost::shared_ptr<DirectGeometryJointZeroOffsets> KukaXmlParser::HandleDirectGeometryJointZeroOffsetsTag(xmlNodePtr node)
  {
    boost::shared_ptr<DirectGeometryJointZeroOffsets> directGeometryJointZeroOffsets(new DirectGeometryJointZeroOffsets());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"joint1");
    directGeometryJointZeroOffsets->joint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint2");
    directGeometryJointZeroOffsets->joint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint3");
    directGeometryJointZeroOffsets->joint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint4");
    directGeometryJointZeroOffsets->joint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return directGeometryJointZeroOffsets;
  }

  boost::shared_ptr<GravityCompensationJointZeroOffsets> KukaXmlParser::HandleGravityCompensationJointZeroOffsetsTag(xmlNodePtr node)
  {
    boost::shared_ptr<GravityCompensationJointZeroOffsets> gravityCompensationJointZeroOffsets(new GravityCompensationJointZeroOffsets());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"joint2");
    gravityCompensationJointZeroOffsets->joint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint3");
    gravityCompensationJointZeroOffsets->joint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"joint4");
    gravityCompensationJointZeroOffsets->joint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return gravityCompensationJointZeroOffsets;
  }

  boost::shared_ptr<GravityCompensationDynamicParameters> KukaXmlParser::HandleGravityCompensationDynamicParametersTag(xmlNodePtr node)
  {
    boost::shared_ptr<GravityCompensationDynamicParameters> gravityCompensationDynamicParameters(new GravityCompensationDynamicParameters());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"g");
    gravityCompensationDynamicParameters->g = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"m1");
    gravityCompensationDynamicParameters->m1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"m2");
    gravityCompensationDynamicParameters->m2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"m3");
    gravityCompensationDynamicParameters->m3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"l1");
    gravityCompensationDynamicParameters->l1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"l2");
    gravityCompensationDynamicParameters->l2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"l3");
    gravityCompensationDynamicParameters->l3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"lc1");
    gravityCompensationDynamicParameters->lc1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"lc2");
    gravityCompensationDynamicParameters->lc2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"lc3");
    gravityCompensationDynamicParameters->lc3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return gravityCompensationDynamicParameters;
  }

  boost::shared_ptr<ExtremeKukaValuesConfig> KukaXmlParser::HandleExtremeKukaValuesTag(xmlNodePtr node)
  {
    boost::shared_ptr<ExtremeKukaValuesConfig> extremeKukaValuesConfig (new ExtremeKukaValuesConfig());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"v-arm-joint-max");
    extremeKukaValuesConfig->maxArmVelocityJoint = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"f-arm-joint-max-j1");
    extremeKukaValuesConfig->maxArmForceJoint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"f-arm-joint-max-j2");
    extremeKukaValuesConfig->maxArmForceJoint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"f-arm-joint-max-j3");
    extremeKukaValuesConfig->maxArmForceJoint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"vx-platform-max");
    extremeKukaValuesConfig->maxPlatformVx = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"vx-platform-min");
    extremeKukaValuesConfig->minPlatformVx = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"vy-platform-max");
    extremeKukaValuesConfig->maxPlatformVy = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"vy-platform-min");
    extremeKukaValuesConfig->minPlatformVy = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"omega-platform-max");
    extremeKukaValuesConfig->maxPlatformOmega = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"omega-platform-min");
    extremeKukaValuesConfig->minPlatformOmega = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return extremeKukaValuesConfig;
  }

  boost::shared_ptr<KukaManagerConfig> KukaXmlParser::ParseKukaConfig(std::string fileName)
  {
    boost::shared_ptr<KukaManagerConfig> kukaConfigPtr;

    xmlDocPtr doc = xmlReadFile(fileName.c_str(), "UTF-8", XML_PARSE_NOBLANKS);

    if (doc == NULL)
    {
      cout << "Utils: Cannot parse document: " << fileName << endl;

      return kukaConfigPtr;
    }

    xmlNodePtr currentNode = xmlDocGetRootElement(doc);

    if (currentNode != NULL && !xmlStrcmp(currentNode->name, (const xmlChar *)"kuka"))
    {
      kukaConfigPtr.reset(new KukaManagerConfig());

      currentNode = currentNode->xmlChildrenNode;

      while (currentNode != NULL)
      {
        if (!xmlStrcmp(currentNode->name, (const xmlChar*)"joint-constraints"))
        {
          kukaConfigPtr->jointConstraintsConfig = HandleJointConstraintsTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar*)"denavit-hartenberg-parameters"))
        {
          kukaConfigPtr->denavitHartenbergParametersConfig = HandleDenavitHartenbergParametersTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar*)"arm-joint-home-positions"))
        {
          kukaConfigPtr->armJointHomePositionConfig = HandleArmJointHomePositionTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar*)"arm-joint-front-positions"))
        {
          kukaConfigPtr->armJointFrontPositionConfig = HandleArmJointFrontPositionTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar*)"direct-geometry-joint-zero-offsets"))
        {
          kukaConfigPtr->directGeometryJointZeroOffsetsConfig = HandleDirectGeometryJointZeroOffsetsTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar*)"gravity-compensation-joint-zero-offsets"))
        {
          kukaConfigPtr->gravityCompensationJointZeroOffsetsConfig = HandleGravityCompensationJointZeroOffsetsTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar*)"gravity-compensation-dynamic-parameters"))
        {
          kukaConfigPtr->gravityCompensationDynamicParametersConfig = HandleGravityCompensationDynamicParametersTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar*)"extreme-kuka-values"))
        {
          kukaConfigPtr->extremeKukaValuesConfig = HandleExtremeKukaValuesTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar*)"controller_parameters"))
        {
          kukaConfigPtr->controllerParametersConfig = HandleControllerParametersTag(currentNode);
        }

        currentNode = xmlNextElementSibling(currentNode);
      }
    }

    xmlFreeDoc(doc);

    return kukaConfigPtr;
  }

  boost::shared_ptr<ControllerParametersConfig> KukaXmlParser::HandleControllerParametersTag(xmlNodePtr node)
  {
    boost::shared_ptr<ControllerParametersConfig> controllerParametersConfig(new ControllerParametersConfig());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"velocity-safety-percent");
    controllerParametersConfig->velocitySafetyPercent = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"force-safety-percent");
    controllerParametersConfig->forceSafetyPercent = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"dead-zone-percent");
    controllerParametersConfig->deadZonePercent = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"velocity-alpha-filter");
    controllerParametersConfig->velocityAlphaFilter = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"velocity-threshold");
    controllerParametersConfig->velocityThreshold = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"integral-gain-velocity-control");
    controllerParametersConfig->integralGainVelocityControl = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-arm-velocity-for-teleoperation");
    controllerParametersConfig->maxArmVelocityForTeleoperation = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"endEffector-x-axis-collision-safety-limit");
    controllerParametersConfig->endEffectorXAxisCollisionSafetyLimit = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return controllerParametersConfig;
  }

} // end namespace Utils
