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

    public boolean Found() {
        boolean found = false;
        if (v_distanceC.getLightDetected() == 1) {
            found = true;
        }
        return found;
    }
    public double getVal(){
        return v_distanceC.getLightDetected();
    }

    public void Follow(String side, Mecanum mec) {
        double d = v_distanceC.getLightDetected();
        if (d == .2) {
            if (side.equals("left")) {
                while (Math.abs(mec.get_wheel_rotations()) <= 2) {
                    mec.run_left(fr, fl, br, bl);
                }
            } else if (side.equals("right")) {
                mec.turn_left(fr, fl, br, bl, 180);
                while (Math.abs(mec.get_wheel_rotations()) <= 2) {
                    mec.run_left(fr, fl, br, bl);
                }
            }
            if (side.equals("left")) {
                mec.turn_left(fr, fl, br, bl, 180);
            } else if (side.equals("right")) {
                mec.turn_right(fr, fl, br, bl, 180);
            }
        }
    }

    //initialization routine



}