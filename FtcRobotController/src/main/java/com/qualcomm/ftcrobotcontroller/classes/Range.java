package com.qualcomm.ftcrobotcontroller.classes;

import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by User on 12/22/2016.
 */

public class Range {
    // instance variables
    // private data
    private UltrasonicSensor rangeSensor = new UltrasonicSensor() {
        @Override
        public double getUltrasonicLevel() {
            return 0;
        }

        @Override
        public String status() {
            return "Connection achieved.";
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return "Connection Received.";
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void close() {

        }
    };
    int yBut = 0;
    // public data

    // constructors

    public Range() {
        // default constructor
        rangeSensor.equals(0);
        yBut = 0;
    }
    public Range(double R, int Y) {
        // fill constructor
        rangeSensor.equals(R);
        yBut = Y;
    }

    // sets
    public void setYBut(int Y) {
        yBut = Y;
    }

    // gets
    public int getyBut() {
        return yBut;
    }

    // methods
    public String connection() {
        if (rangeSensor.getUltrasonicLevel() >= 0) {
            return rangeSensor.getConnectionInfo();
        } else {
            return "Connection interrupted. Try again.";
        }
    }
    public double getRange() {
        return rangeSensor.getUltrasonicLevel();
    }
    public String status() {
        return rangeSensor.status();
    }

    // toString
    public String toString() {
        return  "Distance: " + this.getRange() + "\n" +
                "Connect.  " + this.connection() + "\n" +
                "status:   " + this.status();
    }

    // tester
    private static Range r = new Range();
    public static void main(String[] args) {
        System.out.println(r.toString());
        // TODO make main. test. reconfigure motor controller. fix demo. make sure to have power settings.
    }
}
