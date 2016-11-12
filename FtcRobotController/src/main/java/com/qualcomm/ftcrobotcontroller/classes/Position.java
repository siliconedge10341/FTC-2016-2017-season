package com.qualcomm.ftcrobotcontroller.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.ftcrobotcontroller.classes.Distance;

/**
 * Created by the lord and savior Arko Chaterjee on 9/7/2016.
 */
public class Position {

    double wheelDistance = 0;
    float x;
    float y;
    float fieldmaxX;
    float fieldmaxY;
    Distance robotMovement = new Distance();

    public Position() {
        x = 0;
        y = 0;
        fieldmaxX = 12*12;
        fieldmaxY = 12*12;
        robotMovement = new Distance();
    }

    public void move() {


    }




    //initialization routine



}