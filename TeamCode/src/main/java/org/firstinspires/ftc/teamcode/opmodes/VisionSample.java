package org.firstinspires.ftc.teamcode.opmodes;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;
import org.opencv.core.Size;

/**
 * Linear Vision Sample
 * <p/>
 * Use this in a typical linear op mode. A LinearVisionOpMode allows using
 * Vision Extensions, which do a lot of processing for you. Just enable the extension
 * and set its options to your preference!
 * <p/>
 * Please note that the LinearVisionOpMode is specially designed to target a particular
 * version of the FTC Robot Controller app. Changes to the app may break the LinearVisionOpMode.
 * Should this happen, open up an issue on GitHub. :)
 */
@TeleOp (name = "sensorTest", group = "b")

public class VisionSample extends LinearVisionOpMode {

  ModernRoboticsI2cRangeSensor rangef;
  ModernRoboticsI2cRangeSensor rangesf;
  ModernRoboticsI2cRangeSensor rangesb;
  int frameCount = 0;

  @Override
  public void runOpMode() throws InterruptedException {
    //Wait for vision to initialize - this should be the first thing you do
    waitForVisionStart();

    /**
     * Set the camera used for detection
     * PRIMARY = Front-facing, larger camera
     * SECONDARY = Screen-facing, "selfie" camera :D
     **/

    this.setCamera(Cameras.PRIMARY);

    /**
     * Set the frame size
     * Larger = sometimes more accurate, but also much slower
     * After this method runs, it will set the "width" and "height" of the frame
     **/
    this.setFrameSize(new Size(900, 900));

    /**
     * Enable extensions. Use what you need.
     * If you turn on the BEACON extension, it's best to turn on ROTATION too.
     */
    enableExtension(Extensions.BEACON);         //Beacon detection
    enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
    enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control

    /**
     * Set the beacon analysis method
     * Try them all and see what works!
     */
    beacon.setAnalysisMethod(Beacon.AnalysisMethod.COMPLEX);

    /**
     * Set color tolerances
     * 0 is default, -1 is minimum and 1 is maximum tolerance
     */
    beacon.setColorToleranceRed(0);
    beacon.setColorToleranceBlue(0);


    rotation.setIsUsingSecondaryCamera(false);
    rotation.disableAutoRotate();
    rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

    cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
    cameraControl.setAutoExposureCompensation();

    rangef = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_front");
    rangesf = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_left");
    rangesb = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range_side_right");


    //Wait for the match to begin
    waitForStart();

    //Main loop
    //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
    //This loop will exit once the opmode is closed
    while (opModeIsActive()) {
      //Log a few things
      telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
      telemetry.addData("Beacon Left", beacon.getAnalysis().getStateLeft().toString());
      telemetry.addData("Beacon Right", beacon.getAnalysis().getStateRight().toString());
      telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
      telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
      telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
      telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
      telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
      telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
      telemetry.addData("Frame Counter", frameCount);

      telemetry.addData("cm", "%.2f cm", rangef.getDistance(DistanceUnit.CM));
      telemetry.addData("cm", "%.2f cm", rangesb.getDistance(DistanceUnit.CM));
      telemetry.addData("cm", "%.2f cm", rangesf.getDistance(DistanceUnit.CM));

      //You can access the most recent frame data and modify it here using getFrameRgba() or getFrameGray()
      //Vision will run asynchronously (parallel) to any user code so your programs won't hang
      //You can use hasNewFrame() to test whether vision processed a new frame
      //Once you copy the frame, discard it immediately with discardFrame()
      if (hasNewFrame()) {
        //Get the frame
        Mat rgba = getFrameRgba();
        Mat gray = getFrameGray();

        //Discard the current frame to allow for the next one to render
        discardFrame();

        //Do all of your custom frame processing here
        //For this demo, let's just add to a frame counter
        frameCount++;
      }

      //Wait for a hardware cycle to allow other processes to run
      waitOneFullHardwareCycle();
    }
  }
}