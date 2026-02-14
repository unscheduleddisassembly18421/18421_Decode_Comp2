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
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2DStorage;
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


  double  drive           = 0;        // Desired forward power/speed (-1 to +1)
  double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
  double  turn            = 0;


  Gamepad g1 = new Gamepad();
  Gamepad g2 = new Gamepad();

  Gamepad previousG1 = new Gamepad();
  Gamepad previousG2 = new Gamepad();


  boolean trackingToggle = false;

  public static double power = 0;

  public double slowDown = 1;

  public Pose2d blueStartPose = new Pose2d(65,67,Math.toRadians(180));
  public Pose2d redStartPose = new Pose2d(65,-67,Math.toRadians(180));
  public Pose2d middleShootPose = new Pose2d(63, 0, Math.toRadians(180));

  public enum TargetGoal {
    BLUE, RED
  }

  TargetGoal targetGoal = TargetGoal.RED;

  public ElapsedTime shooterClock = new ElapsedTime();
  public ElapsedTime intakeClock = new ElapsedTime();
  public ElapsedTime goalClock = new ElapsedTime();


  @Override
  public void init() {
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    r = new HwRobot(telemetry, hardwareMap);
    r.init();
  }

  @Override
  public void init_loop() {
    if(gamepad1.dpad_down){
      targetGoal = TargetGoal.BLUE;
    }

    if(gamepad1.dpad_up){
      targetGoal = TargetGoal.RED;
    }

    telemetry.addLine("Press Dpad down for red goal tracking");
    telemetry.addLine("Press Dpad up for blue goal tracking");
    telemetry.addData("target Goal", targetGoal);
  }


  @Override
  public void start() {

    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    telemetry.addData("Status", "Start");
    r.start();
    //FtcDashboard.getInstance().startCameraStream(aprilTag, 0);
    r.turret.init();
    runtime.reset();
    r.drive.localizer.setPose(Pose2DStorage.StordedPose);
    shooterClock.reset();
    intakeClock.reset();
    goalClock.reset();
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

    double yaw = r.drive.localizer.getPose().heading.toDouble();//maybe in radians
    if(gamepad2.yWasPressed()){
      r.drive.recalIMU();
    }
    Pose2d currentPose = r.drive.localizer.getPose();

    Pose2d newPose = new Pose2d(currentPose.position.x, currentPose.position.y, Math.toRadians(180));



    telemetry.addData("Status", "Run Time: " + runtime.toString());

    //NEW CODE
    if (g2.dpad_left && !previousG2.dpad_left){
      r.shiftGoalLeft();
    }
    if (g2.dpad_right && !previousG2.dpad_right){
      r.shiftGoalRight();
    }

          if(g2.x && !previousG2.x){
            switch (targetGoal){
              case RED:
                r.drive.localizer.setPose(redStartPose);
                break;

              case BLUE:
                r.drive.localizer.setPose(blueStartPose);
                break;
            }
          } else if (g2.dpad_down) {
            r.turret.startPosition();
          }
          else{
            switch (targetGoal){
              case RED:
                r.aimTurretRed();
                break;

             case BLUE:
                r.aimTurretBlue();
                break;
            }
          }

        switch (targetGoal){
          case RED:
            r.turret.rightLightRed();
//          r.turret.rightLightRed();
            break;

          case BLUE:
            r.turret.rightLightBlue();
//          r.turret.leftLightBlue();
            break;
        }

      if(gamepad2.left_trigger > 0.3){
        r.intake.intakeMotorOn();
        r.outtake.ballBlocKServoBlock();
      } else if (gamepad2.right_trigger > 0.3) {
        r.intake.intakeMotorOn();
        r.outtake.ballBlockServoStart();
      } else{
        r.intake.intakeMotorOff();
        r.outtake.ballBlocKServoBlock();
      }

      if(g1.right_bumper){
        slowDown = 5;
      }
      else{
        slowDown = 1;
      }

      if(gamepad2.b){
        r.intake.intakeMotorForward();
      }

      if(g1.dpad_up && !previousG1.dpad_up){
        targetGoal = TargetGoal.RED;
      }

      if(g1.dpad_down && !previousG1.dpad_down){
        targetGoal = TargetGoal.BLUE;
      }

    //tune PID controller for turret first, then test the code

      drive  = -gamepad1.left_stick_y/slowDown  ;
      strafe = -gamepad1.left_stick_x/slowDown  ;
      turn   = -gamepad1.right_stick_x/slowDown ;



    // Apply desired axes motions to the drivetrain.
    r.drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(
                    drive,
                    strafe
            ),
            turn
    ));


    r.drive.updatePoseEstimate();

    telemetry.addData("x", currentPose.position.x);
    telemetry.addData("y", currentPose.position.y);
    telemetry.addData("heading (deg)", Math.toDegrees(yaw));
    telemetry.addData("launcher1 motors velocity", r.outtake.getVelocity1());
    telemetry.addData("launcher2 motors velocity", r.outtake.getVelocity2());
    telemetry.addData("target goal", targetGoal);
    telemetry.addData("Tracking toggle", trackingToggle);
    //telemetry.update(); //not needed in a normal opmode

    TelemetryPacket packet = new TelemetryPacket();
    packet.fieldOverlay().setStroke("#3F51B5");
    Drawing.drawRobot(packet.fieldOverlay(), currentPose);
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

}
