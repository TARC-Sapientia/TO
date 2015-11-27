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


#include "HapticXmlParser.h"
#include <boost/lexical_cast.hpp>
#include <string>
#include <cstring>
using namespace std;

namespace Utils
{
  HapticXmlParser::HapticXmlParser() { }

  IHapticXmlParser* getHapticXmlParser()
  {
    return new HapticXmlParser();
  }

  boost::shared_ptr<HapticManagerConfig> HapticXmlParser::ParseHapticManagerConfig(std::string fileName)
  {
    boost::shared_ptr<HapticManagerConfig> hapticManagerConfigPtr;

    xmlDocPtr doc = xmlReadFile(fileName.c_str(), "UTF-8", XML_PARSE_NOBLANKS);

    if (doc == NULL)
    {
      cout << "Utils: Cannot parse document: " << fileName << endl;

      return hapticManagerConfigPtr;
    }

    xmlNodePtr currentNode = xmlDocGetRootElement(doc);

    if (currentNode != NULL && !xmlStrcmp(currentNode->name, (const xmlChar *)"haptic"))
    {
      hapticManagerConfigPtr.reset(new HapticManagerConfig());

      currentNode = currentNode->xmlChildrenNode;

      while (currentNode != NULL)
      {
        if (!xmlStrcmp(currentNode->name, (const xmlChar *)"device"))
        {
          hapticManagerConfigPtr->devices.push_back(HandleDeviceTag(currentNode));
        }

        currentNode = xmlNextElementSibling(currentNode);
      }
    }

    xmlFreeDoc(doc);

    return hapticManagerConfigPtr;
  }

  boost::shared_ptr<DeviceConfig> HapticXmlParser::HandleDeviceTag(xmlNodePtr node)
  {
    boost::shared_ptr<DeviceConfig> deviceConfigPtr(new DeviceConfig());

    xmlChar* tmp;
    tmp = xmlGetProp(node, (const xmlChar *)"name");
    deviceConfigPtr->name = string(reinterpret_cast<const char*>(tmp));

    node = node->xmlChildrenNode;

    while(node != NULL)
    {
      if (!xmlStrcmp(node->name, (const xmlChar*)"constraints"))
      {
        deviceConfigPtr->constraintsConfig = HandleConstraintsTag(node);
      }

      else if (!xmlStrcmp(node->name, (const xmlChar*)"threshold-force-feedback"))
      {
        deviceConfigPtr->thresholdForceFeedbackConfig = HandleThresholdConfigForceFeedbackTag(node);
      }

      else if (!xmlStrcmp(node->name, (const xmlChar *)"scalling-coefficient-force-feedback"))
      {
        xmlChar* tmp = xmlGetProp(node, (const xmlChar *)"scale-hj0");
        deviceConfigPtr->scallingCoefficientForceFeedback.push_back(boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp))));

        tmp = xmlGetProp(node, (const xmlChar *)"scale-hj1");
        deviceConfigPtr->scallingCoefficientForceFeedback.push_back(boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp))));

        tmp = xmlGetProp(node, (const xmlChar *)"scale-hj2");
        deviceConfigPtr->scallingCoefficientForceFeedback.push_back(boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp))));
      }

      else if (!xmlStrcmp(node->name, (const xmlChar*)"log"))
      {
        deviceConfigPtr->logConfig = HandleLogTag(node);
      }
      node = xmlNextElementSibling(node);
    }


    return deviceConfigPtr;
  }

  boost::shared_ptr<ConstraintsConfig> HapticXmlParser::HandleConstraintsTag(xmlNodePtr node)
  {
    boost::shared_ptr<ConstraintsConfig> constraintsConfigPtr(new ConstraintsConfig());

    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"force-alpha-filter");
    constraintsConfigPtr->forceAlphaFilter = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"velocity-alpha-filter");
    constraintsConfigPtr->velocityAlphaFilter = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-force");
    constraintsConfigPtr->maxForce = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-velocity");
    constraintsConfigPtr->maxVelocity = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-x-axis");
    constraintsConfigPtr->maxAxisX = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-x-axis");
    constraintsConfigPtr->minAxisX = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-y-axis");
    constraintsConfigPtr->maxAxisY = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-y-axis");
    constraintsConfigPtr->minAxisY = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-z-axis");
    constraintsConfigPtr->maxAxisZ = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-z-axis");
    constraintsConfigPtr->minAxisZ = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-gimbal1");
    constraintsConfigPtr->maxGimbal1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-gimbal1");
    constraintsConfigPtr->minGimbal1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-joint1");
    constraintsConfigPtr->maxJoint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-joint1");
    constraintsConfigPtr->minJoint1 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-joint2");
    constraintsConfigPtr->maxJoint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-joint2");
    constraintsConfigPtr->minJoint2 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-joint3");
    constraintsConfigPtr->maxJoint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-joint3");
    constraintsConfigPtr->minJoint3 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-joint4");
    constraintsConfigPtr->maxJoint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-joint4");
    constraintsConfigPtr->minJoint4 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"max-joint5");
    constraintsConfigPtr->maxJoint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"min-joint5");
    constraintsConfigPtr->minJoint5 = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return constraintsConfigPtr;
  }

  boost::shared_ptr<ThresholdConfigForceFeedback> HapticXmlParser::HandleThresholdConfigForceFeedbackTag(xmlNodePtr node)
  {
    boost::shared_ptr<ThresholdConfigForceFeedback> thresholdConfigPtr(new ThresholdConfigForceFeedback());

    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"velocity-threshold");
    thresholdConfigPtr->velocityThreshold = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"force-threshold");
    thresholdConfigPtr->forceThreshold = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));

    return thresholdConfigPtr;
  }

  boost::shared_ptr<LogConfig> HapticXmlParser::HandleLogTag(xmlNodePtr node)
  {
    boost::shared_ptr<LogConfig> logConfigPtr(new LogConfig());

    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"log-enabled");
    logConfigPtr->enabled = (!strcmp(reinterpret_cast<const char*>(tmp), "true")) ? true : false;

    tmp = xmlGetProp(node, (const xmlChar *)"log-path");
    logConfigPtr->logPath = string(reinterpret_cast<const char*>(tmp));

    tmp = xmlGetProp(node, (const xmlChar *)"log-name");
    logConfigPtr->logName = string(reinterpret_cast<const char*>(tmp));

    return logConfigPtr;
  }
} // end namespace Utils
