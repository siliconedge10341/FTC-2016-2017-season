package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by evank on 10/14/2016.
 */
public class Distance {

    OpticalDistanceSensor v_sensor_distance;
    private final double wheelDiameter = 0;
    private final double pi = 3.1415962;
    private double wheelRotations = 0;

    public Distance(){

    };


    public double getWheelDistance() {

        double distance = v_sensor_distance.getLightDetected();
        wheelRotations = distance / (wheelDiameter * pi);
        // one wheel rotation is the distance being equal to the circumference of the wheel.

        return wheelRotations;
        //
        // this returns the distance of the light traveling from an object and coming back
        //
        // We want this because we want to know how far for the robot to go when touching
        // the button on the wall. We are aiming for the white box so when we get the distance,
        // we move the robot that distance according to the motor distance achieved. Thus,
        // pressing the buttons and getting points.
        //
    }
}


