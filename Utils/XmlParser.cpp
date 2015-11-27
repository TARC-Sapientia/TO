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


#include "XmlParser.h"
#include <boost/lexical_cast.hpp>
#include <cstring>

using namespace std;

namespace Utils {

  XmlParser::XmlParser() { }

  IXmlParser* getXmlParser()
  {
    return new XmlParser();
  }

  boost::shared_ptr<StatisticsConfig> XmlParser::HandleStatisticsTag(xmlNodePtr node)
  {
    boost::shared_ptr<StatisticsConfig> ret(new StatisticsConfig());
    xmlChar *tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"log-path");
    ret->logPath = string(reinterpret_cast<const char*>(tmp));

    tmp = xmlGetProp(node, (const xmlChar *)"log-name");
    ret->logName = string(reinterpret_cast<const char*>(tmp));

    tmp = xmlGetProp(node, (const xmlChar *)"window");
    ret->window = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"alpha");
    ret->alpha = boost::lexical_cast<float>(string(reinterpret_cast<const char*>(tmp)));

    return ret;
  }

  boost::shared_ptr<StreamConfig> XmlParser::HandleStreamTag(xmlNodePtr node)
  {
    boost::shared_ptr<StreamConfig> ret(new StreamConfig());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"type");
    ret->type = string(reinterpret_cast<const char*>(tmp));

    tmp = xmlGetProp(node, (const xmlChar *)"lport");
    ret->lport = boost::lexical_cast<unsigned short>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"rhost");
    ret->rhost = string(reinterpret_cast<const char*>(tmp));

    tmp = xmlGetProp(node, (const xmlChar *)"rport");
    ret->rport = boost::lexical_cast<unsigned short>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"chunk-size");
    ret->chunkSize = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"receive-buffer-size");
    ret->receiveBufferSize = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"send-queue-length");
    ret->sendQueueLength = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

    if ((node->xmlChildrenNode != NULL) && (!xmlStrcmp(node->xmlChildrenNode->name, (const xmlChar*)"statistics")))
      ret->statistics = HandleStatisticsTag(node->xmlChildrenNode);

    return ret;
  }

  boost::shared_ptr<StreamControlConfig> XmlParser::HandleStreamControlTag(xmlNodePtr node)
  {
    boost::shared_ptr<StreamControlConfig> ret(new StreamControlConfig());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"mode");
    ret->mode = string(reinterpret_cast<const char*>(tmp));

    tmp = xmlGetProp(node, (const xmlChar *)"log-path");
    ret->logPath = string(reinterpret_cast<const char*>(tmp));

    tmp = xmlGetProp(node, (const xmlChar *)"log-name");
    ret->logName = string(reinterpret_cast<const char*>(tmp));

    for (xmlAttrPtr attr = node->properties; NULL != attr; attr = attr->next)
    {
      if (strcmp(reinterpret_cast<const char*>(tmp), "mode"))
        ret->parameters[string(reinterpret_cast<const char*>(attr->name))] = string(reinterpret_cast<const char*>(xmlGetProp(node, attr->name)));
    }

    return ret;
  }

  boost::shared_ptr<VideoCaptureConfig> XmlParser::HandleVideoCaptureTag(xmlNodePtr node)
  {
    boost::shared_ptr<VideoCaptureConfig> ret(new VideoCaptureConfig());
    xmlChar *tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"camera");
    ret->camera = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"width");
    ret->width = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"height");
    ret->height = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

    tmp = xmlGetProp(node, (const xmlChar *)"fps");
    ret->fps = boost::lexical_cast<unsigned short>(string(reinterpret_cast<const char*>(tmp)));

    return ret;
  }

  boost::shared_ptr<RemoteSlaveConfig> XmlParser::HandleRemoteSlaveTag(xmlNodePtr node)
  {
    boost::shared_ptr<RemoteSlaveConfig> ret(new RemoteSlaveConfig());

    node = node->xmlChildrenNode;

    while (node != NULL)
    {
      if (!xmlStrcmp(node->name, (const xmlChar *)"stream"))
        ret->streams.push_back(HandleStreamTag(node));

      node = xmlNextElementSibling(node);
    }

    return ret;
  }

  boost::shared_ptr<HapticConfig> XmlParser::HandleHapticTag(xmlNodePtr node)
  {
    boost::shared_ptr<HapticConfig> hapticConfig(new HapticConfig());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"config-path");
    hapticConfig->configPath = string(reinterpret_cast<const char*>(tmp));
    hapticConfig->hapticManagerConfig = getHapticXmlParser()->ParseHapticManagerConfig(hapticConfig->configPath);

    return hapticConfig;
  }

  boost::shared_ptr<KukaConfig> XmlParser::HandleKukaTag(xmlNodePtr node)
  {
    boost::shared_ptr<KukaConfig> kukaConfig(new KukaConfig());
    xmlChar* tmp;

    tmp = xmlGetProp(node, (const xmlChar *)"config-path");
    kukaConfig->configPath = string(reinterpret_cast<const char*>(tmp));
    kukaConfig->kukaManagerConfig = getKukaXmlParser()->ParseKukaConfig(kukaConfig->configPath);

    return kukaConfig;
  }

  boost::shared_ptr<SlaveConfig> XmlParser::ParseSlaveConfig(string fileName)
  {
    boost::shared_ptr<SlaveConfig> slave;
    xmlDocPtr doc = xmlReadFile(fileName.c_str(), "UTF-8", XML_PARSE_NOBLANKS);

    if (doc == NULL)
    {
      cout << "Utils: Cannot parse document: " << fileName << endl;
      return slave;
    }

    xmlNodePtr currentNode = xmlDocGetRootElement(doc);
    if (currentNode != NULL && !xmlStrcmp(currentNode->name, (const xmlChar *)"slave"))
    {
      slave.reset(new SlaveConfig());
      currentNode = currentNode->xmlChildrenNode;

      while (currentNode != NULL)
      {
        if (!xmlStrcmp(currentNode->name, (const xmlChar *)"stream"))
          slave->streams.push_back(HandleStreamTag(currentNode));
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"slave-stream-control"))
          slave->streamControl = HandleStreamControlTag(currentNode);
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"sync"))
        {
          xmlChar *tmp;

          tmp = xmlGetProp(currentNode, (const xmlChar *)"count");
          slave->syncCount = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

          tmp = xmlGetProp(currentNode, (const xmlChar *)"period");
          slave->syncPeriod = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"control-timer"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"period");
          slave->controlPeriod = boost::lexical_cast<unsigned short>(string(reinterpret_cast<const char*>(tmp)));
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"video-timer"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"period");
          slave->videoPeriod = boost::lexical_cast<unsigned short>(string(reinterpret_cast<const char*>(tmp)));
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"video-capture"))
        {
          slave->videoCapture = HandleVideoCaptureTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"haptic"))
        {
          slave->haptic = HandleHapticTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"kuka"))
        {
          slave->kuka = HandleKukaTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"delay"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"artificial-delay");
          slave->artificialDelay = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"kuka-controller-mode"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"mode");
          string text = string(reinterpret_cast<const char*>(tmp));

          if (text == "CONTROLLER_PHANTOM_KUKA_POSITION_FORCE")
          {
            slave->kukaControllerMode = CONTROLLER_PHANTOM_KUKA_POSITION_FORCE;
          }
          if (text == "CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1")
          {
            slave->kukaControllerMode = CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1;
          }
          if (text == "CONTROLLER_NOVINT_KUKA_POSITION_FORCE")
          {
            slave->kukaControllerMode = CONTROLLER_NOVINT_KUKA_POSITION_FORCE;
          }
          if (text == "CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1")
          {
            slave->kukaControllerMode = CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1;
          }
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"log"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"path");
          slave->logPath = string(reinterpret_cast<const char*>(tmp));
        }

        currentNode = xmlNextElementSibling(currentNode);
      }
    }
    xmlFreeDoc(doc);

    return slave;
  }

  boost::shared_ptr<MasterConfig> XmlParser::ParseMasterConfig(std::string fileName)
  {
    boost::shared_ptr<MasterConfig> master;
    xmlDocPtr doc = xmlReadFile(fileName.c_str(), "UTF-8", XML_PARSE_NOBLANKS);

    if (doc == NULL)
    {
      cout << "Utils: Cannot parse document: " << fileName << endl;
      return master;
    }

    xmlNodePtr currentNode = xmlDocGetRootElement(doc);
    if (currentNode != NULL && !xmlStrcmp(currentNode->name, (const xmlChar *)"master"))
    {
      master.reset(new MasterConfig());
      currentNode = currentNode->xmlChildrenNode;

      while (currentNode != NULL)
      {
        if (!xmlStrcmp(currentNode->name, (const xmlChar *)"remote-slave"))
          master->slave = HandleRemoteSlaveTag(currentNode);
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"sync"))
        {
          xmlChar *tmp;

          tmp = xmlGetProp(currentNode, (const xmlChar *)"count");
          master->syncCount = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));

          tmp = xmlGetProp(currentNode, (const xmlChar *)"period");
          master->syncPeriod = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"control-timer"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"period");
          master->controlPeriod = boost::lexical_cast<unsigned short>(string(reinterpret_cast<const char*>(tmp)));
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"video"))
        {
          xmlChar* tmp;

          tmp = xmlGetProp(currentNode, (const xmlChar *)"max-decoding-queue-length");
          master->maxDecodingQueueLength = boost::lexical_cast<unsigned int>(string(reinterpret_cast<const char*>(tmp)));
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"haptic"))
        {
          master->haptic = HandleHapticTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"kuka"))
        {
          master->kuka = HandleKukaTag(currentNode);
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"delay"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"artificial-delay");
          master->artificialDelay = boost::lexical_cast<double>(string(reinterpret_cast<const char*>(tmp)));
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"kuka-controller-mode"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"mode");
          string text = string(reinterpret_cast<const char*>(tmp));

          if (text == "CONTROLLER_PHANTOM_KUKA_POSITION_FORCE")
          {
            master->kukaControllerMode = CONTROLLER_PHANTOM_KUKA_POSITION_FORCE;
          }
          if (text == "CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1")
          {
            master->kukaControllerMode = CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1;
          }
          if (text == "CONTROLLER_NOVINT_KUKA_POSITION_FORCE")
          {
            master->kukaControllerMode = CONTROLLER_NOVINT_KUKA_POSITION_FORCE;
          }
          if (text == "CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1")
          {
            master->kukaControllerMode = CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1;
          }
        }
        else if (!xmlStrcmp(currentNode->name, (const xmlChar *)"log"))
        {
          xmlChar* tmp = xmlGetProp(currentNode, (const xmlChar *)"path");
          master->logPath = string(reinterpret_cast<const char*>(tmp));
        }

        currentNode = xmlNextElementSibling(currentNode);
      }
    }

    xmlFreeDoc(doc);

    return master;
  }

} // end namespace Utils
