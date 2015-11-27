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


#ifndef I_HAPTIC_XML_PARSER_H
#define I_HAPTIC_XML_PARSER_H

#include <iostream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace Utils
{
    /**
     * Structure holding the minimum and maximum values of the haptic device's joint positions.
    */
    struct ConstraintsConfig
    {
        double forceAlphaFilter;
        double velocityAlphaFilter;

        double maxForce;
        double maxVelocity;

        double maxAxisX;
        double minAxisX;
        double maxAxisY;
        double minAxisY;
        double maxAxisZ;
        double minAxisZ;

        double maxGimbal1;
        double minGimbal1;

        double minJoint1;
        double maxJoint1;

        double minJoint2;
        double maxJoint2;

        double minJoint3;
        double maxJoint3;

        double minJoint4;
        double maxJoint4;

        double minJoint5;
        double maxJoint5;
    };

    /**
     * Structure holding the threshold configuration for force-feedback.
    */
    struct ThresholdConfigForceFeedback
    {
        double velocityThreshold;
        double forceThreshold;
    };

    /**
     * Structure holding the log configuration.
     */
    struct LogConfig
    {
        bool enabled;
        std::string logPath;
        std::string logName;
    };

    /**
     * Structure holding all the configuration for one device.
     */
    struct DeviceConfig
    {
        std::string name;
        boost::shared_ptr<ConstraintsConfig> constraintsConfig;
        std::vector<double> scallingCoefficientForceFeedback;
        boost::shared_ptr<ThresholdConfigForceFeedback> thresholdForceFeedbackConfig;
        boost::shared_ptr<LogConfig> logConfig;
    };

    /**
     * Structure holding the configuration of all connected devices.
     */
    struct HapticManagerConfig
    {
        std::vector<boost::shared_ptr<DeviceConfig> > devices;
    };

    /**
     * Abstract interface definition for the haptic configuration XML parser.
     */
    class IHapticXmlParser
    {
    public:

        /**
        * Used to parse the haptic configuration.
        * @param fileName The file containing the haptic configuration.
        * @return a structure containing the parsed configurations.
        */
        virtual boost::shared_ptr<HapticManagerConfig> ParseHapticManagerConfig(std::string fileName) = 0;

    };

    /**
     * Smart pointer wrapper.
     */
    template <class T>
    class IHapticXmlParserPtr : public boost::shared_ptr<T>
    {
    public:

        /**
         * Constructor.
         */
        IHapticXmlParserPtr(T* p) : boost::shared_ptr<T>(p) { }
    };

    /** Smart pointer type definition. */
    typedef IHapticXmlParserPtr<IHapticXmlParser> HapticXmlParserPtr;

    /**
     * Used to create an XML parser instance.
     * @return the pointer to the newly created XML parser.
     */
    IHapticXmlParser* getHapticXmlParser();

} // end namespace Utils

#endif // I_HAPTIC_XML_PARSER_H
