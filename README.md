# TO
### Networked Control of Mobile Manipulators Including Bilateral Teleoperation
The project implements the teleoperation of a KUKA youBot mobile manipulator using two Sensable Phantom Omni or one Novint Falcon haptic device. The bilateral control is implemented on the first joint of the manipulator.

# General description of the system
In the case of the Sensable Phantom Omni based teleoperation a primary device is used to control the arm KUKA youBot and a secondary device is used to control the KUKA youBot platform. In the case of the Novint Falcon device based teleoperation one device is used to control both the platform and the arm of the KUKA youBot.

Both the master and slave side application starts with synchronizing the clocks of the master and slave side controller. After synchronizing, the teleoperation starts by pressing "Enter".

The motion should be enabled by the human operator.

The role of the buttons in the case of Novint Falcon:
  *  N - Enable mobile platform motion
  *  ^V - Enable arm motion
  *  + - Gripper open/close

The role of the buttons in the case of Sensable Phantom Omni:
  *  Primary haptic light gray button - Enable mobile platform motion
  *  Primary haptic dark gray button - Enable mobile platform motion
  *  Primary haptic both button pressed - Gripper open/close

The teleoperation of the platform: The position of the haptic device (Forward-Backward / Left-Right) generates the velocities of the platform (Longitudinal / Transversal). In the case of the Sensable Phantom device one of the gimbals is used to control the angular velocity. In the case of the Novint Falcon device the grip has to be lifted up or pulled down to generate angular velocity.

The teleoperation of the arm: In the case of the Sensable Phantom the first four joints of the haptic device control directly the positions of the first four joints of the KUKA youBot arm. In the case of the Novint Falcon the position of the grip (Left - Right / Forward - Backward) controls the first two joints of the robot.

The bilateral teleoperation (force feedback) is implemented on the first joint of the youBot arm using the time domain passivity approach.

On the slave side the motion of the slave is captured using an USB WebCamera. The video information is transmitted to the master side and it is displayed to the human operator. The rate of the video transmission is regulated based on the quality of the communication.

The software was validated in test various scenarios including transatlantic teleoperation. For demos and publications, acknowledgement can be found on the project website:
http://www.ms.sapientia.ro/~martonl/MartonL_Research_TE.htm

# License

Teleoperation System for Mobile Manipulators Framework

Copyright (C) 2015 RECONFIGURABLE CONTROL OF ROBOTIC SYSTEMS OVER NETWORKS Project

Robotics and Control Systems Laboratory, Department of Electrical Engineering, Sapientia Hungarian University of Transylvania

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see http://www.gnu.org/licenses/.

# System requirements:
##Install build tools:
  1.  gcc (>= 4.8) http://www.mingw.org/
  2.  Boost libraries (>= 1.55) http://www.boost.org/
  3.  Cmake (>= 2.8) https://cmake.org/
  4.  OpenCV (>= 2.4.8) http://opencv.org/
  5.  XML parser (>=2.9) http://www.xmlsoft.org/
  6.  Git client

## Install the youBot driver:
Download and install the youBot driver from https://github.com/youbot/youbot_driver

## Install Sensable Phantom Omni driver:
Download and install (Note: requires reqistration) the OpenHaptics 3.4 Developer edition and its dependencies from http://dsc.sensable.com/

## Install Novint Falcon driver:
Download and install (Note: requires registration) the Novint SDK and its dependencies from http://www.novint.com/index.php/downloads

# Usage:

#### 1. Create a build folder
``` 
  mkdir build
  cd build
```
#### 2.  Create the project using cmake
```
  cmake <options> ..
```

  Where the options are:
  ```
      -DCMAKE_BUILD_TYPE=
        DEBUG   - For a debug build.
        RELEASE - For a release build
      -DCONFIGURATION=
        MASTER  - For master master side configuration.
        SLAVE   - For slave side configuration.
      -DYOUBOT=     // Only for SLAVE side
        ON  - The build uses the youBot driver.
        OFF - No youBot library is included. The data is generated.
      -DHAPTIC=     // Only for MASTER side
        OFF             - No haptic library is included. The master data is generated.
        PHANTOM_OMNI    - The used haptic device type. If the devices are not connected, generated data will be sent.
        NOVINT_FALCON   - The used haptic device type. If the devices are not connected, generated data will be sent.
  ```
  
#### 3.  Build the project
```
    make
```

#### 4.  Run as administrator
  4.1 The master side:
```
    cd build/Master
    sudo ./Master <path_to_config>
```
  The master side was implemented and tested on Windows 7 32bit using Visual Studio 2010.

  4.2 The slave side:
```
    cd build/Slave
    sudo ./Slave <path_to_config>
```
  The slave side was implemented and tested on the official Remastered Ubuntu Linux 12.04 shipped with the youBot. For details http://www.youbot-store.com/developers/software/operating-systems/remastered-ubuntu-linux

# Configuration:

The system uses the following configuration files:

  1.  master_control_config.xml: Contains the master side configuration. It includes the network configuration, log files locations and teleoperation control mode.
  2.  slave_control_config.xml: Contains the slave side configuration. It includes the network configuration, log files locations and teleoperation control mode.
  3.  kuka_config.xml: Contains the calibration parameters of the KUKA youBot and its control. It includes the joint, velocity and force limits.
  4.  haptic_config.xml: Contains the configuration and calibration parameters of the haptic devices (Sensable Phantom Omni and Novint Falcon). It includes the joint, velocity and force limits.
