package com.qualcomm.ftcrobotcontroller.classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/*
 * Created by the lord and savior Arko Chaterjee on 9/7/2016.
 */
public class LineFollow {
    OpticalDistanceSensor v_distance;


    public LineFollow() {


    }
    double getColor(){

        return v_distance.getLightDetected();
    }


    //initialization routine



}