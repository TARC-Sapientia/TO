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


#include "YouBotControl.hpp"
#include <iostream>

using namespace KUKAYouBot;
using namespace std;
using namespace youbot;
using namespace boost::units;
using namespace boost::numeric::ublas;
using namespace Communication;
using namespace Utils;

YouBotControl::YouBotControl(boost::shared_ptr<Utils::SlaveConfig> slaveConfig)
    : m_SlaveConfig(slaveConfig),
      m_collision_detector(0.0),
      m_alpha_F_collision_detector(0.98),
      m_collision_threshold(0.5),
      m_previous_control_velocity_J1(0.0)
{
    m_ConfigurationDirectory = YouBot_CONFIG_DIR;
    PLATFORM_VX_MAX = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->maxPlatformVx;
    PLATFORM_VY_MAX = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->maxPlatformVy;
    PLATFORM_OMEGA_MAX = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->maxPlatformOmega;

    PLATFORM_VX_MIN = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->minPlatformVx;
    PLATFORM_VY_MIN = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->minPlatformVy;
    PLATFORM_OMEGA_MIN = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->minPlatformOmega;

    JOINT1_MIN = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint1;
    JOINT1_MAX = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint1;

    JOINT2_MIN = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint2;
    JOINT2_MAX = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint2;

    JOINT3_MIN = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint3;
    JOINT3_MAX = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint3;

    JOINT4_MIN = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint4;
    JOINT4_MAX = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint4;

    JOINT5_MIN = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint5;
    JOINT5_MAX = m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint5;

    if (m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE ||
        m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1)
    {
        // Haptic axis that controls the X axis of KUKA platform (longitudinal position, velocity)
        HAPTIC_MAX_X = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxAxisX;
        HAPTIC_MIN_X = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->minAxisX;

        // Haptic axis that controls the Y axis of KUKA platform (lateral position, velocity)
        HAPTIC_MAX_Z = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxAxisZ;
        HAPTIC_MIN_Z = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->minAxisZ;

        // Haptic axis that controls the angular motion KUKA platform (a)
        HAPTIC_MAX_G1 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxGimbal1;
        HAPTIC_MIN_G1 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->minGimbal1;

        HAPTIC_MAX_J1 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxJoint1;
        HAPTIC_MIN_J1 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->minJoint1;

        HAPTIC_MAX_J2 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxJoint2;
        HAPTIC_MIN_J2 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->minJoint2;

        HAPTIC_MAX_J3 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxJoint3;
        HAPTIC_MIN_J3 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->minJoint3;

        HAPTIC_MAX_J4 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxJoint4;
        HAPTIC_MIN_J4 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->minJoint4;

        HAPTIC_MAX_J5 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxJoint5;
        HAPTIC_MIN_J5 = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->minJoint5;

        V_MAX_MASTER = m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxVelocity;
    }
    else // NOVINT Haptic is used in Position or PO-PC-Joint1 mode
    {
        // Haptic axis that controls the X axis of KUKA platform (longitudinal position, velocity)
        HAPTIC_MAX_X = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->maxAxisX;
        HAPTIC_MIN_X = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->minAxisX;

        // Haptic axis that controls the Y axis of KUKA platform (lateral position, velocity)
        HAPTIC_MAX_Z = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->maxAxisZ;
        HAPTIC_MIN_Z = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->minAxisZ;

        // Haptic axis that controls the angular motion KUKA platform (a)
        HAPTIC_MAX_Y = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->maxAxisY;
        HAPTIC_MIN_Y = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->minAxisY;

        HAPTIC_MAX_G1 = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->maxGimbal1;
        HAPTIC_MIN_G1 = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->minGimbal1;

        HAPTIC_MAX_J1 = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->maxJoint1;
        HAPTIC_MIN_J1 = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->minJoint1;

        HAPTIC_MAX_J2 = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->maxJoint2;
        HAPTIC_MIN_J2 = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->minJoint2;

        HAPTIC_MAX_J3 = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->maxJoint3;
        HAPTIC_MIN_J3 = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->minJoint3;

        V_MAX_MASTER = m_SlaveConfig->haptic->hapticManagerConfig->devices[2]->constraintsConfig->maxVelocity;
    }

    jointDenavitHartenbergInputParameters[0].d = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->dJoint1;
    jointDenavitHartenbergInputParameters[0].a = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->aJoint1;
    jointDenavitHartenbergInputParameters[0].alpha = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->alphaJoint1;

    jointDenavitHartenbergInputParameters[1].d = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->dJoint2;
    jointDenavitHartenbergInputParameters[1].a = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->aJoint2;
    jointDenavitHartenbergInputParameters[1].alpha = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->alphaJoint2;

    jointDenavitHartenbergInputParameters[2].d = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->dJoint3;
    jointDenavitHartenbergInputParameters[2].a = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->aJoint3;
    jointDenavitHartenbergInputParameters[2].alpha = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->alphaJoint3;

    jointDenavitHartenbergInputParameters[3].d = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->dJoint4;
    jointDenavitHartenbergInputParameters[3].a = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->aJoint4;
    jointDenavitHartenbergInputParameters[3].alpha = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->alphaJoint4;

    jointDenavitHartenbergInputParameters[4].d = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->dJoint5;
    jointDenavitHartenbergInputParameters[4].a = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->aJoint5;
    jointDenavitHartenbergInputParameters[4].alpha = m_SlaveConfig->kuka->kukaManagerConfig->denavitHartenbergParametersConfig->alphaJoint5;

    V_SAFETY_PERCENT_MAX = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->velocitySafetyPercent;
    F_SAFETY_PERCENT_MAX = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->forceSafetyPercent;

    ARM_JOINT_V_MAX = V_SAFETY_PERCENT_MAX * m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->maxArmVelocityJoint;
    ARM_JOINT_V_MIN = ARM_JOINT_V_MAX;

    Cv = ARM_JOINT_V_MAX/V_MAX_MASTER; // Haptic velocity -> KUKA velocity

    DELTA_DEAD_ZONE_ARM = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->deadZonePercent * ARM_JOINT_V_MAX;

    F_MAX_J1 = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->maxArmForceJoint1; // Nm
    F_MAX_J2 = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->maxArmForceJoint1; // Nm
    F_MAX_J3 = m_SlaveConfig->kuka->kukaManagerConfig->extremeKukaValuesConfig->maxArmForceJoint1; // Nm

    F_MAX_MASTER = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->forceSafetyPercent *
                   m_SlaveConfig->haptic->hapticManagerConfig->devices[0]->constraintsConfig->maxForce; // N

    Cf_J1 = F_MAX_MASTER/F_MAX_J1;
    Cf_J2 = F_MAX_MASTER/F_MAX_J2;
    Cf_J3 = F_MAX_MASTER/F_MAX_J3;

    ALPHA_F = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->velocityAlphaFilter;
    K_I = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->integralGainVelocityControl;
    Velocity_Threshold = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->velocityThreshold;

    END_EFFECTOR_X_AXIS_SAFETY_LIMIT = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->endEffectorXAxisCollisionSafetyLimit;

    MAX_ARM_JOINT_VELOCITY = m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->maxArmVelocityForTeleoperation;

    m_FreezeAngleArmJoint0 = JOINT1_MIN * radian;

    memset(&prevFilteredData, 0, sizeof(prevFilteredData));
        
    // If the position force-architecture isd used, the passivity controller(PO-PC) isn't active
    bool passivityControlEnabled = false;
    if (m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1 ||
        m_SlaveConfig->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1)
    {
        passivityControlEnabled = true;
    }


    if (m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE ||
        m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1)
    {
        m_YouBotSlaveControllerYouBotPtr.reset(
            new Slave_Controller_YouBot(
                m_SlaveConfig->logPath,
                m_SlaveConfig->controlPeriod / 1000.0, // ms to s
                F_MAX_MASTER,
                ARM_JOINT_V_MAX,
                passivityControlEnabled,
                m_SlaveConfig->artificialDelay,
                m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint1,
                m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint1,
                m_SlaveConfig->haptic->hapticManagerConfig->devices[0 /*PHANTOM_1*/]->constraintsConfig->minJoint1,
                m_SlaveConfig->haptic->hapticManagerConfig->devices[0 /*PHANTOM_1*/]->constraintsConfig->maxJoint1,
                m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->integralGainVelocityControl
               ));
    }

    if (m_SlaveConfig->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE ||
        m_SlaveConfig->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1)
    {
        m_YouBotSlaveControllerYouBotPtr.reset(
            new Slave_Controller_YouBot(
               m_SlaveConfig->logPath,
               m_SlaveConfig->controlPeriod / 1000.0, // ms to s
               F_MAX_MASTER,
               ARM_JOINT_V_MAX,
               passivityControlEnabled,
               m_SlaveConfig->artificialDelay,
               m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->minJoint1,
               m_SlaveConfig->kuka->kukaManagerConfig->jointConstraintsConfig->maxJoint1,
               m_SlaveConfig->haptic->hapticManagerConfig->devices[2 /*NOVINT_H1*/]->constraintsConfig->minJoint1,
               m_SlaveConfig->haptic->hapticManagerConfig->devices[2 /*NOVINT_H1*/]->constraintsConfig->maxJoint1,
               m_SlaveConfig->kuka->kukaManagerConfig->controllerParametersConfig->integralGainVelocityControl
               ));
    }
}

void YouBotControl::InitRobotBase()
{
    try
    {
        m_YouBotBasePtr.reset(new YouBotBase("youbot-base", m_ConfigurationDirectory));
        m_YouBotBasePtr->doJointCommutation();

        hasBase = true;
    }
    catch (std::exception& e)
    {
        LOG(warning) << e.what();
        hasBase = false;
    }
}

void YouBotControl::InitRobotManipulator()
{
    try
    {
        m_YouBotManipulatorPtr.reset(new YouBotManipulator("youbot-manipulator", m_ConfigurationDirectory));
        m_YouBotManipulatorPtr->doJointCommutation();
        m_YouBotManipulatorPtr->calibrateManipulator();

        hasArm = true;

        // Limit max joint velocities
        for (int i = 1; i <= 3; i++)
        {
            quantity<angular_velocity> maxVel = MAX_ARM_JOINT_VELOCITY * radian_per_second;

            MaximumPositioningVelocity maxVelParam;
            maxVelParam.setParameter(maxVel);
            m_YouBotManipulatorPtr->getArmJoint(i).setConfigurationParameter(maxVelParam);

            MaximumVelocityToSetPosition maxVelParamSetPos;
            maxVelParamSetPos.setParameter(maxVel);
            m_YouBotManipulatorPtr->getArmJoint(i).setConfigurationParameter(maxVelParamSetPos);
        }
    }

    catch (std::exception& e)
    {
        LOG(warning) << e.what();
        hasArm = false;
    }
}

void YouBotControl::SetBaseReferences(quantity<si::velocity> longitudinalVelocity, quantity<si::velocity> transversalVelocity, quantity<si::angular_velocity> angularVelocity)
{
    if (hasBase == true)
    {
        try
        {
            m_YouBotBasePtr->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        }
        catch (std::exception& e)
        {
            LOG(warning) << "SetBaseReferences: " << e.what();
        }
    }
}

ControlData YouBotControl::ProcessInputData(ControlData masterData)
{
    // Parameters used for scaling
    double a = 0;
    double b = 0;

    KUKA_OPTIONS kukaOption = NO_MOTION;
    if (masterData.force[8] == 1) kukaOption = ENABLE_ARM;
    if (masterData.force[8] == 2) kukaOption = ENABLE_PLATFORM;
    if (masterData.force[8] == 3) kukaOption = TOGGLE_GRIPPER;

    ControlData currentYoubotState = GetCurrentYouBotStates();

    ControlData youbotReferenceState = masterData;

    quantity<si::velocity> longitudinalVelocityReference = 0 * meter_per_second;
    quantity<si::velocity> transversalVelocityReference = 0 * meter_per_second;
    quantity<si::angular_velocity> angularVelocityReference = 0 * radian_per_second;

    // Reset the old angle values so that the arm will not move
    std::vector<youbot::JointAngleSetpoint> armJointAngleReference(5);
    std::vector<youbot::JointVelocitySetpoint> armJointVelocityReference(5);

    armJointAngleReference[0].angle = currentYoubotState.position[3] * radian;    // ArmJoint 1
    armJointAngleReference[1].angle = currentYoubotState.position[4] * radian;    // ArmJoint 2
    armJointAngleReference[2].angle = currentYoubotState.position[5] * radian;    // ArmJoint 3
    armJointAngleReference[3].angle = currentYoubotState.position[6] * radian;    // ArmJoint 4
    armJointAngleReference[4].angle = currentYoubotState.position[7] * radian;    // ArmJoint 5

    armJointVelocityReference[0].angularVelocity = 0 * radian_per_second;    // ArmJoint 1
    armJointVelocityReference[1].angularVelocity = 0 * radian_per_second;    // ArmJoint 2
    armJointVelocityReference[2].angularVelocity = 0 * radian_per_second;    // ArmJoint 3
    armJointVelocityReference[3].angularVelocity = 0 * radian_per_second;    // ArmJoint 4
    armJointVelocityReference[4].angularVelocity = 0 * radian_per_second;    // ArmJoint 5

    // Implements the PO-PC for the base joint
    if (m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE_POPC_ON_J1 ||
        m_SlaveConfig->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1)
    {
        // Reference position computation
        if (youbotReferenceState.position[3] < HAPTIC_MIN_J1) youbotReferenceState.position[3] = HAPTIC_MIN_J1;
        if (youbotReferenceState.position[3] > HAPTIC_MAX_J1) youbotReferenceState.position[3] = HAPTIC_MAX_J1;
        a = (JOINT1_MAX - JOINT1_MIN) / (HAPTIC_MAX_J1 - HAPTIC_MIN_J1);
        b = JOINT1_MAX  - a * HAPTIC_MAX_J1;

        // Position tracking control component
        double armJoint1ReferencePosition_J1 = (a*youbotReferenceState.position[3] + b);
        double armJoint1CurrentPosition_J1 = currentYoubotState.position[3];
        double tracking_component_J1 = K_I * (armJoint1ReferencePosition_J1 - armJoint1CurrentPosition_J1);

        // Convert the robot velocities from rad/s to m/s
        ControlData currentYouBotStateConverted = VelocityConvert(currentYoubotState,1/Cv);

        // Collision detector

        double f_local_J1 = currentYouBotStateConverted.force[3];

        m_collision_detector = m_alpha_F_collision_detector * m_collision_detector +
                (1 - m_alpha_F_collision_detector)*(f_local_J1 - m_previous_control_velocity_J1);

        double forceBefore = currentYoubotState.force[3];

        if (abs(m_collision_detector) < m_collision_threshold) // NO collision detected
        {
             currentYoubotState.force[3] = 0.0; // No force sent back
             currentYouBotStateConverted.force[3] = 0.0;
        }


        // Sets the robot data to PO-PC controller
        m_YouBotSlaveControllerYouBotPtr -> Slave_Controller_YouBot::SetCurrentRobotControlData(currentYouBotStateConverted);

        // Sets the haptic data to PO-PC controller
        m_YouBotSlaveControllerYouBotPtr -> SetCurrentMasterControlData(youbotReferenceState);

        // Calls the PO-PC
        ControlData popcPassifiedData = m_YouBotSlaveControllerYouBotPtr->Slave_Controller_YouBot::GetNewRobotControlData();

        // Energies calculated by the PO-PC of Joint 1 are sent back to master
        currentYoubotState.energy[3] = popcPassifiedData.energy[3];

        // Post-processing of the passified velocity value. Convert the velocity of Joint 1 from m/s to rad/s
        ControlData youbotPassifiedReferenceState = VelocityFilter(VelocityDeadzone(VelocityLimitator(VelocityConvert(popcPassifiedData,Cv))));

        // Stores previously filtered data
        prevFilteredData = youbotPassifiedReferenceState;

        if (kukaOption == ENABLE_ARM)
        {
            // Add tracking component
            youbotPassifiedReferenceState.velocity[3] += tracking_component_J1; // v_ref + K_I*(x_ref-x)

            armJointVelocityReference[0].angularVelocity = youbotPassifiedReferenceState.velocity[3] * radian_per_second; // Arm Joint 1
            m_previous_control_velocity_J1 = youbotPassifiedReferenceState.velocity[3];
        }
    }

    // Indicates a change in gripper state
    bool gripperChangeReference = false;

    if (kukaOption == ENABLE_ARM)
    {
        // Protects the end effector from collision with the rear column
        if (currentYoubotState.position[8] < END_EFFECTOR_X_AXIS_SAFETY_LIMIT)  // endEffector X coordinate
        {
            if (youbotReferenceState.position[3] < HAPTIC_MIN_J1) youbotReferenceState.position[3] = HAPTIC_MIN_J1;
            if (youbotReferenceState.position[3] > HAPTIC_MAX_J1) youbotReferenceState.position[3] = HAPTIC_MAX_J1;
            a = (JOINT1_MAX - JOINT1_MIN) / (HAPTIC_MAX_J1 - HAPTIC_MIN_J1);
            b = JOINT1_MAX  - a * HAPTIC_MAX_J1;
            armJointAngleReference[0] = (a*youbotReferenceState.position[3] + b) * radian;

            // Get armJoint angles
            std::vector< JointSensedAngle > jointAngles;
            m_YouBotManipulatorPtr->getJointData(jointAngles);
            m_FreezeAngleArmJoint0 = jointAngles[0].angle;
        }
        else
        {
            armJointAngleReference[0].angle = m_FreezeAngleArmJoint0;
        }

        if (m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE)
        {
            if (youbotReferenceState.position[4] < HAPTIC_MIN_J2) youbotReferenceState.position[4] = HAPTIC_MIN_J2;
            if (youbotReferenceState.position[4] > HAPTIC_MAX_J2) youbotReferenceState.position[4] = HAPTIC_MAX_J2;
        }
        a = (JOINT2_MAX - JOINT2_MIN) / (HAPTIC_MAX_J2 - HAPTIC_MIN_J2);
        b = JOINT2_MAX  - a * HAPTIC_MAX_J2;
        armJointAngleReference[1].angle = (a*youbotReferenceState.position[4] + b) * radian;

        if (youbotReferenceState.position[5] < HAPTIC_MIN_J3) youbotReferenceState.position[5] = HAPTIC_MIN_J3;
        if (youbotReferenceState.position[5] > HAPTIC_MAX_J3) youbotReferenceState.position[5] = HAPTIC_MAX_J3;
        a = (JOINT3_MAX - JOINT3_MIN) / (HAPTIC_MAX_J3 - HAPTIC_MIN_J3);
        b = JOINT3_MAX  - a * HAPTIC_MAX_J3;
        armJointAngleReference[2].angle = (a*youbotReferenceState.position[5] + b) * radian;

        if (m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE)
        {
            if (youbotReferenceState.position[7] < HAPTIC_MIN_J5) youbotReferenceState.position[7] = HAPTIC_MIN_J5;
            if (youbotReferenceState.position[7] > HAPTIC_MAX_J5) youbotReferenceState.position[7] = HAPTIC_MAX_J5;
            a = (JOINT4_MAX - JOINT4_MIN) / (HAPTIC_MAX_J5 - HAPTIC_MIN_J5);
            b = JOINT4_MAX  - a * HAPTIC_MAX_J5;
            armJointAngleReference[3].angle = (a*youbotReferenceState.position[7] + b) * radian;
        }

        if (m_SlaveConfig->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE ||
            m_SlaveConfig->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE_POPC_ON_J1)
        {
            // Novint controls the first 2 DOFs, the 3rd and 4th KUKA Joints are set to fix positions

            double joint3FrontPosition = m_SlaveConfig->kuka->kukaManagerConfig->armJointFrontPositionConfig->joint3;
            armJointAngleReference[2].angle = joint3FrontPosition * radian;
            m_YouBotManipulatorPtr->getArmJoint(3).setData(armJointAngleReference[2]);

            double joint4FrontPosition = m_SlaveConfig->kuka->kukaManagerConfig->armJointFrontPositionConfig->joint4;
            armJointAngleReference[3].angle = joint4FrontPosition * radian;
            m_YouBotManipulatorPtr->getArmJoint(4).setData(armJointAngleReference[3]);
        }
    }
    else if (kukaOption == ENABLE_PLATFORM)
    {
        if (youbotReferenceState.velocity[0] < HAPTIC_MIN_X) youbotReferenceState.velocity[0] = HAPTIC_MIN_X;
        if (youbotReferenceState.velocity[0] > HAPTIC_MAX_X) youbotReferenceState.velocity[0] = HAPTIC_MAX_X;

        a = (PLATFORM_VX_MAX - PLATFORM_VX_MIN) / (HAPTIC_MAX_X - HAPTIC_MIN_X);
        b = PLATFORM_VX_MAX  - a * HAPTIC_MAX_X;
        longitudinalVelocityReference = (a*youbotReferenceState.velocity[0] + b) * meter_per_second;

        if (youbotReferenceState.velocity[1] < HAPTIC_MIN_Z) youbotReferenceState.velocity[1] = HAPTIC_MIN_Z;
        if (youbotReferenceState.velocity[1] > HAPTIC_MAX_Z) youbotReferenceState.velocity[1] = HAPTIC_MAX_Z;

        a = (PLATFORM_VY_MAX - PLATFORM_VY_MIN) / (HAPTIC_MAX_Z - HAPTIC_MIN_Z);
        b = PLATFORM_VY_MAX  - a * HAPTIC_MAX_Z;
        transversalVelocityReference = (a * youbotReferenceState.velocity[1] + b) * meter_per_second;

        if(youbotReferenceState.velocity[2] < HAPTIC_MIN_G1) youbotReferenceState.velocity[2] = HAPTIC_MIN_G1;
        if(youbotReferenceState.velocity[2] > HAPTIC_MAX_G1) youbotReferenceState.velocity[2] = HAPTIC_MAX_G1;

        a = (PLATFORM_OMEGA_MAX - PLATFORM_OMEGA_MIN) / (HAPTIC_MAX_G1 - HAPTIC_MIN_G1);
        b = PLATFORM_OMEGA_MAX  - a * HAPTIC_MAX_G1;
        angularVelocityReference = (a*youbotReferenceState.velocity[2] + b) * radian_per_second;
    }
    else // The gripper moves
    {
        if (kukaOption == TOGGLE_GRIPPER && previousKukaOption != TOGGLE_GRIPPER)
        {
            gripperChangeReference = true;
        }
    }
    previousKukaOption = kukaOption;

    SetArmReferences(armJointAngleReference, armJointVelocityReference, gripperChangeReference);

    SetBaseReferences(longitudinalVelocityReference, transversalVelocityReference, angularVelocityReference);

    // Scales youBot Position/Velocity to Haptic Position/Velocity
    currentYoubotState.velocity[3] /= Cv;
    currentYoubotState.velocity[4] /= Cv;
    currentYoubotState.velocity[5] /= Cv;

    currentYoubotState.position[3] /= Cv;
    currentYoubotState.position[4] /= Cv;
    currentYoubotState.position[5] /= Cv;

    return currentYoubotState;
}

ControlData YouBotControl::VelocityConvert(ControlData data, double coefficient)
{
    data.velocity[3]= data.velocity[3] * coefficient;
    return data;
}

ControlData YouBotControl::VelocityFilter(ControlData data)
{
    data.velocity[3]=ALPHA_F*prevFilteredData.velocity[3] + (1-ALPHA_F)*data.velocity[3];
    return data;
}

ControlData YouBotControl::VelocityLimitator(ControlData data)
{
    if(data.velocity[3] < -ARM_JOINT_V_MAX)
    {
        data.velocity[3] = -ARM_JOINT_V_MAX;
    }
    else if(data.velocity[3] > ARM_JOINT_V_MAX)
    {
        data.velocity[3] = ARM_JOINT_V_MAX;
    }
    return data;
}

ControlData YouBotControl::VelocityDeadzone(ControlData data)
{
    if(abs(data.velocity[3]) < DELTA_DEAD_ZONE_ARM)
    {
        data.velocity[3] = 0;
    }
    return data;    
}

void YouBotControl::SetArmReferences(std::vector<youbot::JointAngleSetpoint>& referenceAngle, std::vector<youbot::JointVelocitySetpoint>& referenceVelocity, bool referenceGripperChange)
{
    if (hasArm == true)
    {
        try
        {
            // Note the armJoints are numbered from 1

            if (m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE ||
                m_SlaveConfig->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE)

            {
                m_YouBotManipulatorPtr->getArmJoint(1).setData(referenceAngle[0]);
            }
            else // PO-PC active
            {
                m_YouBotManipulatorPtr->getArmJoint(1).setData(referenceVelocity[0]);
            }

            m_YouBotManipulatorPtr->getArmJoint(2).setData(referenceAngle[1]);
            m_YouBotManipulatorPtr->getArmJoint(3).setData(referenceAngle[2]);
            m_YouBotManipulatorPtr->getArmJoint(4).setData(referenceAngle[3]);
            m_YouBotManipulatorPtr->getArmJoint(5).setData(referenceAngle[4]);

            if (referenceGripperChange == true)
            {
                if (m_IsGripperOpened == false)
                {
                    m_YouBotManipulatorPtr->getArmGripper().open();
                    m_IsGripperOpened = true;
                }
                else
                {
                    m_YouBotManipulatorPtr->getArmGripper().close();
                    m_IsGripperOpened = false;
                }
            }
        }
        catch (std::exception& e)
        {
            LOG(warning) << "SetArmReferences: " << e.what();
        }
    }
}

ControlData YouBotControl::GetCurrentYouBotStates()
{
    ControlData data;
    memset(&data, 0, sizeof(data));

    if (hasBase == true)
    {
        // Fill data with base position values
        quantity< si::length > longitudinalPosition;
        quantity< si::length > transversalPosition;
        quantity< plane_angle >  orientation;

        m_YouBotBasePtr->getBasePosition(longitudinalPosition, transversalPosition, orientation);

        data.position[0] = longitudinalPosition.value();
        data.position[1] = transversalPosition.value();
        data.position[2] = orientation.value();

        // Fill data with base velocity values
        quantity< si::velocity > longitudinalVelocity;
        quantity< si::velocity > transversalVelocity;
        quantity< si::angular_velocity >  angularVelocity;

        m_YouBotBasePtr->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);

        data.velocity[0] = longitudinalVelocity.value();
        data.velocity[1] = transversalVelocity.value();
        data.velocity[2] = angularVelocity.value();
    }

    if (hasArm == true)
    {
        // Get armJoint angles
        std::vector< JointSensedAngle > jointAngles;
        m_YouBotManipulatorPtr->getJointData(jointAngles);

         // Fill data with armJoint position values
        data.position[3] = jointAngles[0].angle.value();
        data.position[4] = jointAngles[1].angle.value();
        data.position[5] = jointAngles[2].angle.value();
        data.position[6] = jointAngles[3].angle.value();
        data.position[7] = jointAngles[4].angle.value();

        // Calculate gravity compensations
        std::vector<double> gravityTorque = CalculateGravityCompensation(data.position[4], data.position[5], data.position[6]);

        // Used to calculated direct geometry
        std::vector<double> positions(5, 0); // initialize vector, 5 elements with zero
        positions[0] = data.position[3];
        positions[1] = data.position[4];
        positions[2] = data.position[5];
        positions[3] = data.position[6];
        positions[4] = data.position[7];

        std::vector<double> endEffector = CalculateDirectGeometry(positions);

        // Fill with endEffector position
        data.position[8] = endEffector[0];
        data.velocity[8] = endEffector[1];
        data.force[8] = endEffector[2];

        // Get armJoint velocities
        std::vector< JointSensedVelocity > jointVelocities;
        m_YouBotManipulatorPtr->getJointData(jointVelocities);

        // Fill data with armJoint velocity values
        data.velocity[3] = jointVelocities[0].angularVelocity.value();
        data.velocity[4] = jointVelocities[1].angularVelocity.value();
        data.velocity[5] = jointVelocities[2].angularVelocity.value();
        data.velocity[6] = jointVelocities[3].angularVelocity.value();
        data.velocity[7] = jointVelocities[4].angularVelocity.value();

        // Get armJoint torque values
        std::vector< JointSensedTorque > jointTorques;
        m_YouBotManipulatorPtr->getJointData(jointTorques);

        // Fill data with armJoint torque values
        if (m_SlaveConfig->kukaControllerMode == CONTROLLER_PHANTOM_KUKA_POSITION_FORCE ||
            m_SlaveConfig->kukaControllerMode == CONTROLLER_NOVINT_KUKA_POSITION_FORCE)
        {
            // Position-force control
            data.force[3] = jointTorques[0].torque.value();
            data.force[4] = jointTorques[1].torque.value() - gravityTorque[0];
            data.force[5] = jointTorques[2].torque.value() - gravityTorque[1];
            data.force[6] = jointTorques[3].torque.value() - gravityTorque[2];
            data.force[7] = jointTorques[4].torque.value();
        }
        else
        {
            // Velocity-force control
            data.force[3] = jointTorques[0].torque.value() * Cf_J1;
            data.force[4] = 0.0;
            data.force[5] = 0.0;
            data.force[6] = 0.0;
            data.force[7] = 0.0;
        }
    }

    return data;
}

void YouBotControl::SendArmToHomePosition()
{   
    if (hasArm == true)
    {
        youbot::JointAngleSetpoint armAngles[5];

        armAngles[0].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointHomePositionConfig->joint1 * radian;
        m_YouBotManipulatorPtr->getArmJoint(1).setData(armAngles[0]);

        armAngles[1].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointHomePositionConfig->joint2 * radian;
        m_YouBotManipulatorPtr->getArmJoint(2).setData(armAngles[1]);

        armAngles[2].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointHomePositionConfig->joint3 * radian;
        m_YouBotManipulatorPtr->getArmJoint(3).setData(armAngles[2]);

        armAngles[3].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointHomePositionConfig->joint4 * radian;
        m_YouBotManipulatorPtr->getArmJoint(4).setData(armAngles[3]);

        armAngles[4].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointHomePositionConfig->joint5 * radian;
        m_YouBotManipulatorPtr->getArmJoint(5).setData(armAngles[4]);

        LOG(info) << "Moving arm to home position.";
    }
}

void YouBotControl::SendArmToFrontPosition()
{
    if (hasArm == true)
    {
        youbot::JointAngleSetpoint armAngles[5];

        armAngles[0].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointFrontPositionConfig->joint1 * radian;
        m_YouBotManipulatorPtr->getArmJoint(1).setData(armAngles[0]);

        armAngles[1].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointFrontPositionConfig->joint2 * radian;
        m_YouBotManipulatorPtr->getArmJoint(2).setData(armAngles[1]);

        armAngles[2].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointFrontPositionConfig->joint3 * radian;
        m_YouBotManipulatorPtr->getArmJoint(3).setData(armAngles[2]);

        armAngles[3].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointFrontPositionConfig->joint4 * radian;
        m_YouBotManipulatorPtr->getArmJoint(4).setData(armAngles[3]);

        armAngles[4].angle = m_SlaveConfig->kuka->kukaManagerConfig->armJointFrontPositionConfig->joint5 * radian;
        m_YouBotManipulatorPtr->getArmJoint(5).setData(armAngles[4]);

        LOG(info) << "Moving arm to front position.";
    }
}

std::vector<double> YouBotControl::CalculateDirectGeometry(std::vector<double> jointPositions)
{
    std::vector<double> endEffectorPosition(3, 0.0);

    matrix<double> joint_dh_matrix(4,4); // DH matrix returns this

    matrix<double> dataMatrix(4,4);

    for (int i=0; i<4; i++)
        for (int j=0; j<4; j++)
        {
            if (i == j)
                dataMatrix(i,j) = 1;
            else
                dataMatrix(i,j) = 0;
        }

    double theta_0_offset_ref = m_SlaveConfig->kuka->kukaManagerConfig->directGeometryJointZeroOffsetsConfig->joint1;
    double theta_1_offset_ref = m_SlaveConfig->kuka->kukaManagerConfig->directGeometryJointZeroOffsetsConfig->joint2;
    double theta_2_offset_ref = m_SlaveConfig->kuka->kukaManagerConfig->directGeometryJointZeroOffsetsConfig->joint3;
    double theta_3_offset_ref = m_SlaveConfig->kuka->kukaManagerConfig->directGeometryJointZeroOffsetsConfig->joint4;

    double theta0_offset = theta_0_offset_ref * 3.1415 / 180;
    double theta1_offset = theta_1_offset_ref * 3.1415 / 180;
    double theta2_offset = theta_2_offset_ref * 3.1415 / 180;
    double theta3_offset = theta_3_offset_ref * 3.1415 / 180;

    jointPositions[0] = theta0_offset + jointPositions[0];
    jointPositions[1] = theta1_offset + jointPositions[1];
    jointPositions[2] = theta2_offset + jointPositions[2];
    jointPositions[3] = theta3_offset + jointPositions[3];

    for (int i = 0; i <= 4; i++)
    {
        if (i == 3)
        {
            joint_dh_matrix = CalculateDenavitHartenbergMatrix(jointPositions[i] + 3.1415 / 2,
                                    jointDenavitHartenbergInputParameters[i].d,
                                    jointDenavitHartenbergInputParameters[i].a,
                                    jointDenavitHartenbergInputParameters[i].alpha);
        }
        else
        {
            joint_dh_matrix = CalculateDenavitHartenbergMatrix(jointPositions[i],
                                    jointDenavitHartenbergInputParameters[i].d,
                                    jointDenavitHartenbergInputParameters[i].a,
                                    jointDenavitHartenbergInputParameters[i].alpha);
        }

        dataMatrix = prod(dataMatrix, joint_dh_matrix);
    }

    endEffectorPosition[0] = dataMatrix(0,3);
    endEffectorPosition[1] = dataMatrix(1,3);
    endEffectorPosition[2] = dataMatrix(2,3);

    return endEffectorPosition;
}

matrix<double> YouBotControl::CalculateDenavitHartenbergMatrix(double theta, double d, double a, double alpha)
{
    matrix<double> dh_matrix(4,4);

    dh_matrix(0,0) = cos(theta);
    dh_matrix(0,1) = -sin(theta)*cos(alpha);
    dh_matrix(0,2) = sin(theta)*sin(alpha);
    dh_matrix(0,3) = a*cos(theta);

    dh_matrix(1,0) = sin(theta);
    dh_matrix(1,1) = cos(theta)*cos(alpha);
    dh_matrix(1,2) = -cos(theta)*sin(alpha);
    dh_matrix(1,3) = a*sin(theta);

    dh_matrix(2,0) = 0;
    dh_matrix(2,1) = sin(alpha);
    dh_matrix(2,2) = cos(alpha);
    dh_matrix(2,3) = d;

    dh_matrix(3,0) = 0;
    dh_matrix(3,1) = 0;
    dh_matrix(3,2) = 0;
    dh_matrix(3,3) = 1;

    return dh_matrix;
}

std::vector<double> YouBotControl::CalculateGravityCompensation(double armJointTheta1, double armJointTheta2, double armJointTheta3)
{
    std::vector<double> forces(3,0.0);

    double g = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->g;

    double m1 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->m1;
    double m2 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->m2;
    double m3 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->m3;

    double l1 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->l1;
    double l2 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->l2;
    double l3 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->l3;

    double lc1 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->lc1;
    double lc2 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->lc2;
    double lc3 = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationDynamicParametersConfig->lc3;

    double theta_1_offset_ref = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationJointZeroOffsetsConfig->joint2;
    double theta_2_offset_ref = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationJointZeroOffsetsConfig->joint3;
    double theta_3_offset_ref = m_SlaveConfig->kuka->kukaManagerConfig->gravityCompensationJointZeroOffsetsConfig->joint4;

    double theta1_offset = theta_1_offset_ref * 3.1415 / 180;
    double theta2_offset = theta_2_offset_ref * 3.1415 / 180;
    double theta3_offset = theta_3_offset_ref * 3.1415 / 180;

    double theta1 = theta1_offset + armJointTheta1;
    double theta2 = theta2_offset + armJointTheta2;
    double theta3 = theta3_offset + armJointTheta3;

    if (hasArm == true)
    {
        forces[0] = g*(m1 * lc1 * cos(theta1) + m2* l1 * cos(theta1) + m2 * lc2 * cos(theta1+theta2) + m3 * l1 * cos(theta1) + m3 * l2 * cos(theta1+theta2) + m3 * lc3 * cos(theta1+theta2+theta3));
        forces[1] = g*(m2 * lc2 * cos(theta1+theta2) + m3 * l2 * cos(theta1+theta2) + m3 * lc3 * cos(theta1+theta2+theta3));
        forces[2] = g*(m3 * lc3 * cos(theta1+theta2+theta3));
    }
    return forces;
}
