# RMDX ROS Driver

## Install

    vcs .. import < my.repos

## usage

edit this `motor.beginn("/dev/ttyUSB0", 115200, 0x00);` in the **rmdx_test.cpp** file. Parameters are: port, baud, ID

    rosrun rmdx_driver rmdx_test

The program will send vel commands in form of a sin wave to the motor causing it alternating between clock and counter clock wise rotation.
