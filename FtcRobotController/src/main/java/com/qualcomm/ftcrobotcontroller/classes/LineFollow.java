package com.qualcomm.ftcrobotcontroller.classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/*
 * Created by the lord and savior Arko Chaterjee on 9/7/2016.
 */
public class LineFollow {
    OpticalDistanceSensor v_distanceC;
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;

    public LineFollow() {
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void Follow(String side, Mecanum mec, Distance v_distance) {
        double d = v_distanceC.getLightDetected();
        if (d == 1) {
            if (side.equals("left")) {
                while (v_distance.getWheelDistance() <= 2) {
                    mec.run_left(fr, fl, br, bl);
                }
            }
        }
    }

    //initialization routine



}