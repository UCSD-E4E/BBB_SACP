# BeagleBone Black Stabilized Aerial Camera Platform

## Notes
* BBB has <linux/i2c-dev.h>.  No need to include local copy.

## Command Protocol
* Set setpoint
* * SETPOINTRRRPPPYYY
* * * RRR is the rotation around the roll axis in integer degrees from [0, 360), zero-padded to three digits.
* * * PPP is the rotation around the pitch axis in integer degrees from [0, 360), zero-padded to three digits.
* * * YYY is the rotation around the yaw axis in integer degres from [0, 360), zero-padded to three digits.
* Enable/disable stabilization
* * STABILIZATIONX
* * * X is the stabilization flag.  Set X to 1 if enable stabilization, 0 otherwise.
