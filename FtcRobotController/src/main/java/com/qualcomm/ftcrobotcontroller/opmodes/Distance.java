package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.ftcrobotcontroller.opmodes.PushBotHardwareSensors;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by evank on 10/14/2016.
 */
public class Distance extends PushBotHardwareSensors {

    OpticalDistanceSensor v_sensor_distance;

    public Distance() {
        //
        // Initialize Distance variables
        //
        v_sensor_distance = hardwareMap.opticalDistanceSensor.get("v_sensor_obs");
    }

    public double getDistance() {
        return v_sensor_distance.getLightDetected();
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
