# BeagleBone Black Stabilized Aerial Camera Platform

## Overview

This software is designed to control the Stabilized Aerial Camera Platform (SACP) from a BeagleBone Black.  This software is designed to run directly on a BeagleBone Black, RevB.  It will interface directly with the following hardware:

* 2x MPU9150
* 3x Fixed Rotation Servos

This software will take the orientations from both IMUs (one located in the sensor frame and one located in the body frame) and stabilize the sensor frame relative to the local NED (North East Down) frame.  To do this, the software will take the orientation of the sensor frame relative to the local NED frame as a quaternion and the orientation of the body frame relative to the local NED frame as a quarternion and find the error to the setpoint.  This error will be used to determine the proper inputs to give the servos so that the sensor frame is stable relative to the local NED frame.  The advantage to using this method over dead reckoning is that we can account for any offset due to servo loading.
