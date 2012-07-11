MultiWii 2.1 Clone
==============

This is a customized version of the MultiWii software (http://www.multiwii.com). I try to keep this as close to the official releases as possible.

It includes the following changes:
 * config.h changes for my quadrocopter (and MAXCHECK change in MultiWii.ino, because my TX doesn't go up to 1900)
 * experimental MARG attitude algorithm in IMU_EXPERIMENTAL.ino (activate via "#define IMU_EXPERIMENTAL" in config.h)
 * correct angle calculation in default getEstimatedAttitude() method
 * additional MSP_SUPERRAW_IMU command in SERIAL.ino to get non-smoothed sensor values
 

Goals
-------------------

 * staying compatible with existing MultiWii verions (currently 2.1 and its release candidates)
 * robust attitude estimation code (at the expense of some precious bytes, currently 3650)
 * better altitude estimation code and altitude hold mode
 * shrinking code size (binary form), so everything still fits in a Arduino Pro Mini (328P)
 
 
P.S.: I don't take any responsibility if this code leads to a crash of your model. 