/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Variables.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.INTAKE_OFF_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.RELOAD_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.SHOOTER_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.firstAngle;
import static org.firstinspires.ftc.teamcode.Variables.firstShootingAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondShootingAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdShootingAngle;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.HwRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;


/*
 * Demonstrates an empty iterative OpMode
 */
@Config
@TeleOp(name = "Driver Control", group = "Concept")
//@Disabled
public class DriverControl extends OpMode {
  //test
  public HwRobot r = null;
  private ElapsedTime runtime = new ElapsedTime();
  public static double headingErrorBlue = 0;
  public static double headingErrorRed = 0;

  private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
  private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
  //NEED SPECIFIC APRIL TAG

  private VisionPortal visionPortal;               // Used to manage the video source.
  private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
  private AprilTagDetection blueDesiredTag = null;
  private AprilTagDetection redDesiredTag = null;
  boolean blueTargetFound     = false;// Used to hold the data for a detected AprilTag
  boolean redTargetFound    = false;


  public static double TURN_GAIN   =  0.04  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)


  public static double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
  public static double DESIRED_DISTANCE = 12.0;
  double  drive           = 0;        // Desired forward power/speed (-1 to +1)
  double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
  double  turn            = 0;

  public static double RED_BEARING_OFFSET = 5;
  public static double BLUE_BEARING_OFFSET = 1;

  Gamepad g1 = new Gamepad();
  Gamepad g2 = new Gamepad();

  Gamepad previousG1 = new Gamepad();
  Gamepad previousG2 = new Gamepad();

  boolean intakeToggle = false;

  boolean shooterToggle = false;

  boolean slideToggle = false;

  boolean ballBlockToggle = false;

  public static double power = 0;

  public double slowDown = 1;

  public enum ShooterState {
    READY, FARFIRE1, FARFIRE2, FARFIRE3, NEARFIRE1, NEARFIRE2, NEARFIRE3, RELOAD
  }


  public enum IntakeState {
    READY, INTAKE1, INTAKE2, INTAKE3, FULL, FIRING
  }

  public enum GreenPosition{
    RIGHT, MIDDLE, LEFT
  }

  ShooterState shooterState = ShooterState.READY;
  IntakeState intakeState = IntakeState.READY;
  GreenPosition greenPosition;

  public ElapsedTime shooterClock = new ElapsedTime();
  public ElapsedTime intakeClock = new ElapsedTime();


  @Override
  public void init() {
    r = new HwRobot(telemetry, hardwareMap);
    r.init();
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    telemetry.addData("Status", "Initialized");
    initAprilTag();
    //FtcDashboard.getInstance().startCameraStream(aprilTag, 0);
  }

  @Override
  public void init_loop() {
    if(gamepad2.dpad_left){
      greenPosition = GreenPosition.LEFT;
    }
    if(gamepad2.dpad_right){
      greenPosition = GreenPosition.RIGHT;
    }
    if(gamepad2.dpad_down){
      greenPosition = GreenPosition.MIDDLE;
    }
    telemetry.addData("green position", greenPosition);
  }


  @Override
  public void start() {
    runtime.reset();
    shooterClock.reset();
    intakeClock.reset();
  }

  /**
   * This method will be called repeatedly during the period between when
   * the START button is pressed and when the OpMode is stopped.
   */
  @Override
  public void loop() {
    previousG2.copy(g2);
    previousG1.copy(g1);
    g1.copy(gamepad1);
    g2.copy(gamepad2);

    double yaw = r.drive.localizer.getPose().heading.toDouble(); //maybe in radians



    telemetry.addData("Status", "Run Time: " + runtime.toString());

    //NEW CODE
    if(g1.aWasPressed()){
      shooterToggle = !shooterToggle;
    }

    if(g1.bWasPressed()){
      intakeToggle = !intakeToggle;
    }

    if(g1.yWasPressed()){
      ballBlockToggle = !ballBlockToggle;
    }

    if(g1.xWasPressed()){
      slideToggle = !slideToggle;
    }

    if(shooterToggle){
      r.outtake.launcherMotor1OnFar();
      r.outtake.launcherMotor2OnFar();
    }
    else {
      r.outtake.launcherMotor1Off();
      r.outtake.launcherMotor2Off();
    }

    if(intakeToggle){
      r.intake.intakeMotorOn();
    }
    else {
      r.intake.intakeMotorOff();
    }

    if(ballBlockToggle){
      r.outtake.ballBlocKServoBlock();
    }
    else{
      r.outtake.ballBlockServoStart();
    }

    if(slideToggle){
      r.intake.slideServoExtend();
    }
    else{
      r.intake.slideServostart();
    }



    r.turret.readColorSensors();

    blueTargetFound = false;
    redTargetFound = false;
    blueDesiredTag  = null;
    redDesiredTag = null;

    // Step through the list of detected tags and look for a matching tag
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    for (AprilTagDetection detection : currentDetections) {
      // Look to see if we have size info on this tag.
      if (detection.metadata != null) {
        //  Check to see if we want to track towards this tag.
        if (detection.id == 20) {
          // Yes, we want to use this tag.
          blueTargetFound = true;
          blueDesiredTag = detection;
          break;  // don't look any further.
        }
        else if (detection.id == 24) {
          redTargetFound = true;
          redDesiredTag = detection;
          break;
        }else {
          // This tag is in the library, but we do not want to track it right now.
          telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
        }
      } else {
        // This tag is NOT in the library, so we don't have enough information to track to it.
        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
      }
    }

    // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
    if (gamepad1.left_bumper && (blueTargetFound || redTargetFound)) {
      if (blueTargetFound) {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        //double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = blueDesiredTag.ftcPose.bearing + BLUE_BEARING_OFFSET;
        telemetry.addData("bearing", headingError);
        telemetry.addData("bearing offset", BLUE_BEARING_OFFSET);
        headingErrorBlue = blueDesiredTag.ftcPose.bearing + BLUE_BEARING_OFFSET;
        //double  yawError        = desiredTag.ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        //drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        //strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        r.turret.leftLightNewColor();
      }
      else if(redTargetFound){
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        //double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = redDesiredTag.ftcPose.bearing - RED_BEARING_OFFSET;
        telemetry.addData("bearing", headingError);
        telemetry.addData("bearing offset", RED_BEARING_OFFSET);
        headingErrorRed = redDesiredTag.ftcPose.bearing - RED_BEARING_OFFSET;
        //double  yawError        = desiredTag.ftcPose.yaw;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        //drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        //strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        r.turret.leftLightNewColor();
      }
    }
    else {

      // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
      drive  = -gamepad1.left_stick_y/slowDown  ;  // Reduce drive rate to 50%.
      strafe = -gamepad1.left_stick_x/slowDown  ;  // Reduce strafe rate to 50%.
      turn   = -gamepad1.right_stick_x/slowDown ;
      r.turret.leftLightRed();// Reduce turn rate to 33%.
      //telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
    }
//    telemetry.update();



    // Apply desired axes motions to the drivetrain.
    r.drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(
                    drive,
                    strafe
            ),
            turn
    ));


    r.drive.updatePoseEstimate();
    r.turret.update();

    Pose2d pose = r.drive.localizer.getPose();
    telemetry.addData("x", pose.position.x);
    telemetry.addData("y", pose.position.y);
    telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
    telemetry.addData("motors at velocity", r.outtake.launchMotorsAtVelocity());
    telemetry.addData("launcher1 motors velocity", r.outtake.getVelocity1());
    telemetry.addData("launcher2 motors velocity", r.outtake.getVelocity2());
    telemetry.addData("green Position", greenPosition);
    telemetry.addData("close motors at velocity", r.outtake.launcherMotorsAtVelocityNear());
    telemetry.addData("bearing error", RED_BEARING_OFFSET);
    telemetry.addData("blue tag detected state", blueTargetFound);
    telemetry.addData("red tag detection data", redTargetFound);
    telemetry.addData("blue heading error", headingErrorBlue);
    telemetry.addData("red heading error", headingErrorRed);
    telemetry.update();

    TelemetryPacket packet = new TelemetryPacket();
    packet.fieldOverlay().setStroke("#3F51B5");
    Drawing.drawRobot(packet.fieldOverlay(), pose);
    FtcDashboard.getInstance().sendTelemetryPacket(packet);

  }

  /**
   * This method will be called once, when this OpMode is stopped.
   * <p>
   * Your ability to control hardware from this method will be limited.
   */
  @Override
  public void stop() {

  }
  public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
      lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
      Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
      Utils.matToBitmap(frame, b);
      lastFrame.set(b);
      return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
      // do nothing
    }

    @Override
    public void getFrameBitmap(Continuation<? extends org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>> continuation) {
      continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
  }

  private void initAprilTag() {
    // Create the AprilTag processor by using a builder.
    aprilTag = new AprilTagProcessor.Builder().build();

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // e.g. Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    aprilTag.setDecimation(3);


    // Create the vision portal by using a builder.
    if (USE_WEBCAM) {
      visionPortal = new VisionPortal.Builder()
              .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
              .addProcessor(aprilTag)
              .build();
      CameraStreamProcessor processor = new CameraStreamProcessor();
      FtcDashboard.getInstance().startCameraStream(processor, 0);
    }
  }


}
