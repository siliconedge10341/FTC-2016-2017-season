package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.classes.Mecanum;

import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import org.firstinspires.ftc.teamcode.classes.Range;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.classes.ProjectileMotion;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="MecanumDrive", group ="Drive")
public class MecanumDriver extends OpMode{
    // instance variables
    // private variables
    // Motors
	private DcMotor motorCollector;
	private DcMotor motorFR;
	private DcMotor motorFL;
	private DcMotor motorBR;
	private DcMotor motorBL;
	private DcMotor motorShootL;
	private DcMotor motorShootR;
    private DcMotor motorLS;

    // Servos
	private Servo ballRelease;
    private Servo leftClamp;
    private Servo rightClamp;

	private int percision_flag = 0;
    // percision flag is to decrease power, if power is decreased, the robot will go slower.
	private double ballpos = .5;
    private double LSRotations = 0;
	private Mecanum yo = new Mecanum();
    private double power = 0;
    private Range dist = new Range();

    public MecanumDriver() {
		percision_flag = 0;
        LSRotations = 0;
	}

	@Override
	public void init() {
        // Initialize everything
        //dist.setRange(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "projectile_distance"));  - This line is broken, not sure why - Dival
		motorFL = hardwareMap.dcMotor.get("fl_motor");
		motorFR = hardwareMap.dcMotor.get("fr_motor");
		motorBL = hardwareMap.dcMotor.get("bl_motor");
		motorBR = hardwareMap.dcMotor.get("br_motor");
        motorLS = hardwareMap.dcMotor.get("linear_slide_motor");

		motorCollector = hardwareMap.dcMotor.get("ball_collector");

		motorShootL = hardwareMap.dcMotor.get("shooter_left");
		motorShootR = hardwareMap.dcMotor.get("shooter_right");

		ballRelease = hardwareMap.servo.get("servo_ball");
		ballRelease.setPosition(ballpos);
        leftClamp = hardwareMap.servo.get("servo_left_clamp");
        rightClamp = hardwareMap.servo.get("servo_right_clamp");
	}

	//main function body
	@Override
	public void loop() {
        /*   Start-A Runs the Program
        *-----GAMEPAD FUNCTION KEY-----
        *-GAMEPAD-
        *UP: Increase Decrease Power
        *RIGHT: Move Servo Right
        *LEFT: Move Servo Left
        * DOWN: None;
        *-BUTTONS-//
        * A: Turns on Collector
        * B: Runs Shooter
        * X: Raise Slide
        * Y: Lower Slide
        *-TRIGGERS-//
        * LEFT: None;
        * RIGHT: None;
        */

        // Decreases Power
		if (gamepad1.dpad_up) {
			percision_flag++;
		}

		if (percision_flag >= 2) {
			percision_flag = 0;
		}

        // runs the robot
		yo.set_Power(gamepad1.right_stick_x, gamepad1.left_stick_y, gamepad1.left_stick_x, percision_flag);
		yo.run_motor(motorFR, motorFL, motorBR, motorBL);

        // Run Collector
		if (gamepad1.a)  {
			motorCollector.setPower(0.75);
		} else {
			motorCollector.setPower(0);
		}

		//bantu shooter
		if (ballpos <= Servo.MIN_POSITION + .25 || ballpos >= Servo.MAX_POSITION - .25 || gamepad2.b){
            if (dist.getData() < .5 /*Meters*/) {
                power = 1.0;
            } else if (dist.getData() >= .5 /*Meters*/ && dist.getData() < 1 /*Meters*/) {
                power = 1.25;
            } else if (dist.getData() >= 1 /*Meters*/) {
                power = 1.5;
            } else {
                power = 0.75;
            }
			motorShootL.setPower(power);
			motorShootR.setPower(-power);
		} else {
			motorShootL.setPower(0);
			motorShootR.setPower(0);
		}

        // Moves Servo
		if (gamepad2.dpad_right){
            if (ballpos > Servo.MIN_POSITION) {
                ballpos = ballpos -.05;
            } else {
                // Nothing
            }
		}else if(gamepad2.dpad_left){
            if (ballpos < Servo.MAX_POSITION) {
                ballpos = ballpos +.05;
            } else {
                // Nothing
            }
		}
        ballRelease.setPosition(ballpos);

        // Raises CatBall
        if (gamepad2.x && LSRotations <= 40) {
            motorLS.setPower(0.5);
            LSRotations++;
        } else if (gamepad2.y && LSRotations >= 1) {
            motorLS.setPower(-0.5);
            LSRotations--;
        } else {
            motorLS.setPower(0.0);
        }

        // Clamps CatBall

	}

	@Override
	public void stop() {
		yo.set_Power(0, 0, 0,0);
		// set to zero so the power doesn't influnce any motion or rotation in the robot
		yo.run_motor(motorFR, motorFL, motorBR, motorBL);
	}


}