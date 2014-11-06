# BeagleBone Black Stabilized Aerial Camera Platform

## Overview

This software is designed to control the Stabilized Aerial Camera Platform (SACP) from a BeagleBone Black.  This software is designed to run directly on a BeagleBone Black, RevB.  It will interface directly with the following hardware:

* 2x MPU9150
* 3x Fixed Rotation Servos

This software will take the orientations from both IMUs (one located in the sensor frame and one located in the body frame) and stabilize the sensor frame relative to the local NED (North East Down) frame.  To do this, the software will take the orientation of the sensor frame relative to the local NED frame as a quaternion and the orientation of the body frame relative to the local NED frame as a quarternion and find the error to the setpoint.  This error will be used to determine the proper inputs to give the servos so that the sensor frame is stable relative to the local NED frame.  The advantage to using this method over dead reckoning is that we can account for any offset due to servo loading.

## Software Requirements

This software directly requires the following libraries:

* libzmq (0MQ)

## Topology

BBB_SACP consists of two distinct processes: the control process and the UI process.  The control process is responsible for managing the sensor, servos, and control loops, and provides a socket interface to control setpoint, stabilization mode, and tracking mode.  The UI process is responsible for managing the translation from user bump commands to a setpoint for the gimbal.

The two processes will be linked using two links: a control link and a data link.  The control link will be a ZMQ REQ-REP socket pair, with the UI having the REQ socket and the controller having the REP socket.  The UI sends a message to the controller requesting the controller to set a new setpoint or control mode, and the controller replies with a message when the gimbal has reached the setpoint, or when the controller has determined that the gimbal has failed.  The data link will consist of a ZMQ PUB-SUB socket pair.  The controller will constantly publish a data structure denoting its current state (orientation, geolocation, state) for the controller to accept and use at will.  This also allows for other programs to listen and process the data for other operations, such as monitoring coverage.

NOTE: The data link may eventually be replaced with a MAVlink stream, or be wrapped with a MAVlink stream to enable control using MissionPlanner or QGroundControl.  This functionality may also be implemented as a UI proxy.

## Setpoint Serial Control Protocol
Physical Layer: RS232

Data Layer: Three fields, tab delimited.  Frame delimited with newline.  Fields are: Roll, Pitch, Yaw in degrees, no padding, zero centered (allow negatives), ASCII encoding, sent by master device.  Slave acknowledges with \ACK\LF.  When slave reaches setpoint (or deadband around setpoint), slave acknowledges with "STPTRCHD\n".  Master is required to not acknowledge transmission from slave.
