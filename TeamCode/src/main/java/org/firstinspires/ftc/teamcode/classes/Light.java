package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Evan Krimpenfort on 10/14/2016.
 */

public class Light {

    private OpticalDistanceSensor v_sensor;

    // constructors
    public Light(){
        // Default Constructor
        // Initialize everything.
    }

    public void setV_sensor(OpticalDistanceSensor Sens1) {
        v_sensor = Sens1;
    }

    public double getLightDetected() {
        return v_sensor.getLightDetected();
    }

    public double getRaw() {
        return v_sensor.getRawLightDetected();
    }
}


