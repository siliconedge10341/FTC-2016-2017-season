/*
Modern Robotics Range Sensor Example
Created 9/8/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.x Beta
Reuse permitted with credit where credit is due

Configuration:
I2cDevice on an Interface Module named "range" at the default address of 0x28 (0x14 7-bit)

This program can be run without a battery and Power Destitution Module.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Range {
    // instance variables
    // private variables
    private ModernRoboticsI2cRangeSensor range;

    // constructors
    public Range() {
        // default constructor
        // initialize range
    }

    // sets
    public void setRange(ModernRoboticsI2cRangeSensor RANG1){
        range = RANG1;
    }

    // gets
    public double getData() {
        return range.getDistance(DistanceUnit.CM);
    }

    // methods // None //
    // toString // None //
}
