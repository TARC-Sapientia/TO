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


#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotJoint.hpp"
#include "youbot/YouBotGripper.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "Slave_Controller_YouBot.h"
#include "Slave_Controller.h"
#include "CommunicationData.h"
#include <string>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/shared_ptr.hpp>
#include "XmlParser.h"

namespace KUKAYouBot
{
    /**
     * Stores the constant Denavit-Hartenberg (DH) parameters of a youBot joint.
     */
    struct DenavitHartenbergParameters
    {
        /** DH parameter: Offset along previous z to the common normal. */
        double d;

        /** DH parameter: Length of the common normal. */
        double a;

        /** DH parameter: Angle about common normal, from old z axis to new z axis. */
        double alpha;
    };

    /**
     * Stores the motion options of the youBot. These options are set by the buttons of the applied haptic device.
     */
    enum KUKA_OPTIONS
    {
        /** Neither the arm nor the platform moves. */
        NO_MOTION = 0,

        /** Only the arm moves. */
        ENABLE_ARM = 1,

        /** Only the platform moves. */
        ENABLE_PLATFORM = 2,

        /** The gripper state changes. */
        TOGGLE_GRIPPER = 3
    };

	/** Implements the control of the KUKA youBot mobile manipulator based on the signals received from the master side haptic devices. */
    class YouBotControl
    {
        public:
			/**
			 * Constructor
			 * Sets the values of configuration parameters that are necessary for robot control.
			 * @param[in] slaveConfig Object that stores the configuration parameters for the robot control. The parameters are obtained from a configuration file.
			 */
            YouBotControl(boost::shared_ptr<Utils::SlaveConfig> slaveConfig);
			
			/** Initializes the control of the youBot's platform. */
            void InitRobotBase();

			/** Initializes the control of the youBot's arm. Sets the maximum velocities for each joint. */
            void InitRobotManipulator();

			/** Sends the reference velocity commands for the youBot platform controller. 
			* @param[in] longitudinalVelocity Reference velocity for longitudinal platform motion.
			* @param[in] transversalVelocity Reference velocity for transversal platform motion.
			* @param[in] angularVelocity Reference angular velocity for platform rotation.
			*/
            void SetBaseReferences(quantity<si::velocity> longitudinalVelocity, quantity<si::velocity> transversalVelocity, quantity<si::angular_velocity> angularVelocity);

			/** Sends the reference position or velocity commands for the youBot arm controller.
			* Position reference is used in the case of position force architecture.
			* The velocity reference is used if bilateral PO-PC control is applied on the corresponding joint.
			* Also sets the gripper state (open/closed).
			* @param[in] referenceAngle Reference angular positions for all the joints.
			* @param[in] referenceVelocity Reference velocity for transversal platform motion.
			* @param[in] referenceGripperChange Reference angular velocity for platform rotation.
			*/
            void SetArmReferences(std::vector<youbot::JointAngleSetpoint>& referenceAngle, std::vector<youbot::JointVelocitySetpoint>& referenceVelocity, bool referenceGripperChange);

			/** Position control for the youBot arm. The reference joints positions correspond to the home position of the youBot arm. */
            void SendArmToHomePosition();

            /** Position control for the youBot arm. The reference joints positions move the gripper in the front of the platform. Used for bilateral control experiments. */
            void SendArmToFrontPosition();

			/** Gets all the youBot variables (position, velocity, force) of the robot's platform and arm (all joint)
			* The returned forces from joint 1,2,3 are gravity compensated
			* return ControlData type structure that contains all the robot variables.
			*/
            Communication::ControlData GetCurrentYouBotStates();
            
			/** Processes the data received from the master side haptic device and using these inputs computes the position and force references for the robot.
			* Performs the calibration of the velocity and position signals for the arm and the platform.
			* Based on the received haptic button states enables/disables the robot's motions and controls the grippers.
			* When it is selected implements the passivity controller on the base joint of the arm.
			* @param[in] masterData Control data structure that contains the positions, forces and velocities received from the haptic device.
			* return Control data structure that contains the reference positions and velocities for the youBot platform and arm.
			*/
            Communication::ControlData ProcessInputData(Communication::ControlData masterData);

    private:
            /** Computes the gravity torques that act on the youBot joints 1,2,3.
            * Necessary for gravity compensation in force feedback used for bilateral control.
            * No gravity compensation is calculated for youBot's first joint (arm base joint 0)
            * @param[in] Angular position of the first joint
            * @param[in] Angular position of the second joint relative to armJointTheta1
            * @param[in] Angular position of the third joint relative to armJointTheta2
            * return Vector that contains the three computed gravity torques.
            */
            std::vector<double> CalculateGravityCompensation(double armJointTheta1, double armJointTheta2, double armJointTheta3);

            /** Computes a Denavit Hartenberg transformation matrix from the i-1'th joint to the i'th joint
            * @param[in] theta Angle about previous z, from old x to new x
            * @param[in] d Offset along previous z to the common normal
            * @param[in] a Length of the common normal
            * @param[in] alpha Angle about common normal, from old z axis to new z axis
            * return Transformation matrix
            */
            boost::numeric::ublas::matrix<double> CalculateDenavitHartenbergMatrix(double theta, double d, double a, double alpha);

            /** Computes the end effector's position of an 5 DOF robot using the Denavit Hartenberg formalism.
            * It multiplies DOF number of transformation matrices.
            * @param[in] jointPositions Vector that contains joint variables of the robot
            * return Vector that contains x,y,z coordinates of the end effector.
            */
            std::vector<double> CalculateDirectGeometry(std::vector<double> jointPositions);

			/** Scales the velocity value of the youBot base joint of the arm in a Control data structure.
			* Function used for pre- or post-processing the velocity signals around the time domain passivity controller.
			* @param[in] data Control data structure that contains the velocity value to be converted
			* @param[in] coefficient Velocity scaling coefficient.
            * return Control data structure that contains the scaled velocity value.
			*/
            Communication::ControlData VelocityConvert(Communication::ControlData data, double coefficient);

            /** Applies a low pass filter to the velocity value of the youBot's arm base joint in a Control data structure.
			* Function used for post-processing the velocity signals around the time domain passivity controller.
			* @param[in] data Control data structure that contains the velocity value to be filtered
            * return Control data structure that contains the filtered velocity value.
			*/
            Communication::ControlData VelocityFilter(Communication::ControlData data);

            /** Saturates the absolute value of the velocity value of the youBot's arm base joint of the arm in a Control data structure.
			* Function used for post-processing the velocity signals around the time domain passivity controller.
			* @param[in] data Control data structure that contains the velocity value to be saturated
            * return Control data structure that contains the filtered velocity value.
			*/
            Communication::ControlData VelocityLimitator(Communication::ControlData data);

			/** Sets the velocity value of the youBot's arm base joint of the arm to zero under a predefined threshold of the youBot's arm base joint of the arm in a Control data structure.
			* Function used for post-processing the velocity signals around the time domain passivity controller.
			* @param[in] data Control data structure that contains the velocity value to be set
            * return Control data structure that contains the modified velocity value.
			*/
            Communication::ControlData VelocityDeadzone(Communication::ControlData data);
            
			/** Pointer to an object that directly controls the robot platform. */
            boost::shared_ptr<youbot::YouBotBase> m_YouBotBasePtr;
			
			/** Pointer to an object that directly controls the robot arm. */
            boost::shared_ptr<youbot::YouBotManipulator> m_YouBotManipulatorPtr;
			
			/** Pointer to an object that implements the slave side time domain passivity controller. */
            boost::shared_ptr<Slave_Controller_YouBot> m_YouBotSlaveControllerYouBotPtr;
			
			/** True if the arm is detected by the youBot initialization function. */
            bool hasArm;

            /** True if the platform is detected by the youBot initialization function. */
            bool hasBase;

            /** Stores the location of the configuration files. */
            std::string m_ConfigurationDirectory;

            /** Limit value for the arm's base joint position to avoid collision with the sensor's pole mounted on the back side of the platform. */
            quantity<plane_angle> m_FreezeAngleArmJoint0;
			
			/** Limit value for the arm's end effector position along x axis to avoid collision with the sensor's pole mounted on the back side of the platform. */
            double END_EFFECTOR_X_AXIS_SAFETY_LIMIT;

            /** Minimum value of joint 1 (base joint) position. */
            double JOINT1_MIN;
			
            /** Maximum value of joint 1 (base joint) position. */
            double JOINT1_MAX;

            /** Minimum value of joint 2 position. */
            double JOINT2_MIN;

            /** Maximum value of joint 2 position. */
            double JOINT2_MAX;

            /** Minimum value of joint 3 position. */
            double JOINT3_MIN;

            /** Maximum value of joint 3 position. */
            double JOINT3_MAX;

            /** Minimum value of joint 4 position. */
            double JOINT4_MIN;

            /** Maximum value of joint 4 position. */
            double JOINT4_MAX;

            /** Minimum value of joint 5 position. */
            double JOINT5_MIN;

            /** Maximum value of joint 5 position. */
            double JOINT5_MAX;

            /** Minimum value of joint 1 (base joint) velocity. */
            double ARM_JOINT_V_MIN;

            /** Maximum value of joint 1 (base joint) velocity. */
            double ARM_JOINT_V_MAX;

            /** Minimum value of the platform's longitudinal velocity. */
            double PLATFORM_VX_MIN;

            /** Maximum value of the platform's longitudinal velocity. */
            double PLATFORM_VX_MAX;

            /** Minimum value of the platform's transversal velocity. */
            double PLATFORM_VY_MIN;

            /** Maximum value of the platform's transversal velocity. */
            double PLATFORM_VY_MAX;

            /** Minimum value of the platform's angular velocity. */
            double PLATFORM_OMEGA_MIN;

            /** Maximum value of the platform's angular velocity. */
            double PLATFORM_OMEGA_MAX;

            /** Maximum value of joint 1 (base joint) torque. */
            double F_MAX_J1;

            /** Maximum value of joint 2 torque. */
            double F_MAX_J2;

            /** Maximum value of joint 3 torque. */
            double F_MAX_J3;

            /** Maximum value of the haptic device tool's position along the x axis. */
            double HAPTIC_MAX_X;

            /** Minimum value of the haptic device tool's position along the x axis. */
            double HAPTIC_MIN_X;

            /** Maximum value of the haptic device tool's position along the y axis. */
            double HAPTIC_MAX_Y;

            /** Minimum value of the haptic device tool's position along the y axis. */
            double HAPTIC_MIN_Y;

            /** Maximum value of the haptic device tool's position along the z axis. */
            double HAPTIC_MAX_Z;

            /** Minimum value of the haptic device tool's position along the z axis. */
            double HAPTIC_MIN_Z;

            /** Maximum value of the haptic device tool's gimbal angle. */
            double HAPTIC_MAX_G1;

            /** Minimum value of the haptic device tool's first glimbal angle. */
            double HAPTIC_MIN_G1;

            /** Maximum value of the haptic device's joint 1. */
            double HAPTIC_MAX_J1;

            /** Minimum value of the haptic device's joint 1. */
            double HAPTIC_MIN_J1;

            /** Maximum value of the haptic device's joint 2. */
            double HAPTIC_MAX_J2;

            /** Minimum value of the haptic device's joint 2. */
            double HAPTIC_MIN_J2;

            /** Maximum value of the haptic device's joint 3. */
            double HAPTIC_MAX_J3;

            /** Minimum value of the haptic device's joint 3. */
            double HAPTIC_MIN_J3;

            /** Maximum value of the haptic device's joint 4. */
            double HAPTIC_MAX_J4;

            /** Minimum value of the haptic device's joint 4. */
            double HAPTIC_MIN_J4;

            /** Maximum value of the haptic device's joint 5. */
            double HAPTIC_MAX_J5;

            /** Minimum value of the haptic device's joint 5. */
            double HAPTIC_MIN_J5;

            /** Maximum value of the haptic device's velocity. */
            double V_MAX_MASTER;

            /** Maximum value of the haptic device's force. */
            double F_MAX_MASTER;

            /** Velocity scaling coefficient for the youBot's joints. */
            double Cv;

            /** Force scaling coefficient for haptic device's force from the youBot arm joint 1 (base joint) force. */
            double Cf_J1;

            /** Force scaling coefficient for haptic device's force from the youBot arm joint 2 force. */
            double Cf_J2;

            /** Force scaling coefficient for haptic device's force from the youBot arm joint 3 force. */
            double Cf_J3;

            /** The percent of maximum joint velocities of youBot manipulator for safe teleoperation. */
            double V_SAFETY_PERCENT_MAX;

            /** The percent of maximum joint forces of the haptic device for safe teleoperation */
            double F_SAFETY_PERCENT_MAX;

            /** Lower dead zone value for the arm joint velocity. Applied after the time domain passivity controller. */
            double DELTA_DEAD_ZONE_ARM;

            /** Filter coefficient for velocity. Applied after the time domain passivity controller. */
            double ALPHA_F;

            /** Integral constant for velocity based time domain passivity controller to achieve position tracking. */
            double K_I;

            /** Filtered collision detector signal */
            double m_collision_detector;

            /** Filter coefficient for collision detector */
            double m_alpha_F_collision_detector;

            /** Treshold for collision detection */
            double m_collision_threshold;

            /** The command velocity for joint 1 in the previous control sample. Used by the collision detector */
            double m_previous_control_velocity_J1;

            /** Lower threshold value for the velocity. The time domain passivity controller is active only when the absolute velocity is over this value. */
            double Velocity_Threshold;

            /** User defined maximum value for the joint velocities that can be safely applied to teleoperation. */
            double MAX_ARM_JOINT_VELOCITY;

			/** Stores the current motion options of the youBot. */
            KUKA_OPTIONS previousKukaOption;

            /** Status of the gripper (opened = true). */
            bool m_IsGripperOpened;

            /** Stores the youBot states in the previous . Used to implement the dynamic velocity filter. */
            Communication::ControlData prevFilteredData;

            /** Array that stores the Denavit Hartenberg parameters of the youBot's arm. */
            DenavitHartenbergParameters jointDenavitHartenbergInputParameters[5];
			
			/** Stores all the configuration parameter of the youBot. */
            boost::shared_ptr<Utils::SlaveConfig> m_SlaveConfig;
    };
}
