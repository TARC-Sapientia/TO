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


#ifndef I_KUKA_XML_PARSER_H
#define I_KUKA_XML_PARSER_H

#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>

namespace Utils
{
  /**
   * Minimum and maximum values of the KUKA youBot arm's joint positions
   */
  struct JointConstraints
  {
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
   * The constant Denavit Hartenberg parameters (d, a, alpha) of the KUKA youBot arm
   */
  struct DenavitHartenbergParameters
  {
      double dJoint1;
      double aJoint1;
      double alphaJoint1;

      double dJoint2;
      double aJoint2;
      double alphaJoint2;

      double dJoint3;
      double aJoint3;
      double alphaJoint3;

      double dJoint4;
      double aJoint4;
      double alphaJoint4;

      double dJoint5;
      double aJoint5;
      double alphaJoint5;
  };

  /**
   * Constant reference joints positions corresponding to the home position of the youBot arm
   */
  struct ArmJointHomePosition
  {
      double joint1;
      double joint2;
      double joint3;
      double joint4;
      double joint5;
  };

  /**
   * Constant reference joints positions in which the youBot arm's gripper is in the front of the platform.
   * Used for bilateral control experiments.
   */
  struct ArmJointFrontPosition
  {
      double joint1;
      double joint2;
      double joint3;
      double joint4;
      double joint5;
  };

  /**
   * Offsets to joint positions to compute the end effector's position using the Denavit Hartenberg method
   */
  struct DirectGeometryJointZeroOffsets
  {
      double joint1;
      double joint2;
      double joint3;
      double joint4;
  };

  /**
   * Offsets to joint positions to compute the joint position dependent gravity torques acting on each segment
   */
  struct GravityCompensationJointZeroOffsets
  {
      double joint2;
      double joint3;
      double joint4;
  };

  /**
   * Dynamic model parameters of the KUKA youBot arm to compute the gravity compensation terms
   * g - gravitaional acceleration
   * m - mass of a segment
   * l - length of the segment
   * lc - position of the center of gravity of a segment
   */
  struct GravityCompensationDynamicParameters
  {
      double g;

      double m1;
      double m2;
      double m3;

      double l1;
      double l2;
      double l3;

      double lc1;
      double lc2;
      double lc3;
  };

  /**
   * Extreme values for the KUKA youBot velocities and forces
   * \see YouBotControl
   */
  struct ExtremeKukaValuesConfig
  {
      double maxArmVelocityJoint;

      double maxArmForceJoint1;
      double maxArmForceJoint2;
      double maxArmForceJoint3;

      double maxPlatformVx;
      double minPlatformVx;
      double maxPlatformVy;
      double minPlatformVy;

      double maxPlatformOmega;
      double minPlatformOmega;
  };

  /**
   * Parameters to impelemt the bilateral control of the KUKA youBot
   * \see YouBotControl
   */
  struct ControllerParametersConfig
  {
      double velocitySafetyPercent;
      double forceSafetyPercent;
      double deadZonePercent;
      double velocityAlphaFilter;
      double velocityThreshold;
      double integralGainVelocityControl;
      double maxArmVelocityForTeleoperation;
      double endEffectorXAxisCollisionSafetyLimit;
  };

  /**
   * Data structure containing all the KUKA youBot configuration parameters.
   */
  struct KukaManagerConfig
  {
      boost::shared_ptr<JointConstraints> jointConstraintsConfig;
      boost::shared_ptr<DenavitHartenbergParameters> denavitHartenbergParametersConfig;
      boost::shared_ptr<ArmJointHomePosition> armJointHomePositionConfig;
      boost::shared_ptr<ArmJointFrontPosition> armJointFrontPositionConfig;
      boost::shared_ptr<DirectGeometryJointZeroOffsets> directGeometryJointZeroOffsetsConfig;
      boost::shared_ptr<GravityCompensationDynamicParameters> gravityCompensationDynamicParametersConfig;
      boost::shared_ptr<GravityCompensationJointZeroOffsets> gravityCompensationJointZeroOffsetsConfig;
      boost::shared_ptr<ExtremeKukaValuesConfig> extremeKukaValuesConfig;
      boost::shared_ptr<ControllerParametersConfig> controllerParametersConfig;
  };

  /**
   * Abstract interface definition for the KUKA youBot configuration XML parser.
   */
  class IKukaXmlParser
  {
    public:

      /**
       * Used to parse the Kuka youBot's configuration.
       * @param fileName The file containing the slave configuration.
       * @return a structure containing the parsed configurations.
       */
      virtual boost::shared_ptr<KukaManagerConfig> ParseKukaConfig(std::string fileName) = 0;
  };

  /**
   * Smart pointer wrapper.
   */
  template <class T>
  class IKukaXmlParserPtr : public boost::shared_ptr<T>
  {
    public:

      /**
       * Constructor.
       */
      IKukaXmlParserPtr(T* p) : boost::shared_ptr<T>(p) { }
  };

  /** Smart pointer type definition. */
  typedef IKukaXmlParserPtr<IKukaXmlParser> KukaXmlParserPtr;

  /**
   * Used to create an XML parser instance.
   * @return the pointer to the newly created XML parser.
   */
  IKukaXmlParser* getKukaXmlParser();

} // end namespace Utils

#endif // I_KUKA_XML_PARSER_H
