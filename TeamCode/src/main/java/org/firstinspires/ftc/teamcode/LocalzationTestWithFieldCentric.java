package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config

@TeleOp(name = "localzation test: field centric", group = "robot")
public class LocalzationTestWithFieldCentric extends LinearOpMode {

    Gamepad g1 = new Gamepad();
    Gamepad previousG1 = new Gamepad();
    public double yaw;
    double headingLockValue = 0;

    boolean headingLock;
    public static double error;
    public static double kp = 0.05;
    public double stickSensitivity = 0.05;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            double slowdown = 1;

            previousG1.copy(g1);
            g1.copy(gamepad1);

            yaw = drive.localizer.getPose().heading.toDouble();

            if (gamepad1.right_bumper){
                slowdown = 3;
            }

            if(Math.abs(g1.right_stick_x) < stickSensitivity && Math.abs(previousG1.right_stick_x) > stickSensitivity){
                headingLock = true;
                headingLockValue = yaw;
            }

            if(Math.abs(g1.right_stick_x) > stickSensitivity){
                headingLock = false;
            }
            if (headingLock){
                error = headingLockValue - yaw;
            }
            else {
                error = 0;
            }


            double forward = -gamepad1.left_stick_y/slowdown;
            double right = gamepad1.left_stick_x/slowdown;
            double rotate = gamepad1.right_stick_x+(error*kp);

            driveFieldRelative(forward, right, rotate, drive);




//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ),
//                    -gamepad1.right_stick_x
//            ));

            drive.updatePoseEstimate();

            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(yaw));
            telemetry.addData("press and hold right bumper for slow mode", slowdown);
            telemetry.addData("heading lock", headingLock);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    private void driveFieldRelative(double forward, double right, double rotate, MecanumDrive drive) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);//gives me angle between x and y
        double r = Math.hypot(right, forward);//gives me the hypotonuse b/w 0,0 and x, y

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                drive.localizer.getPose().heading.toDouble());

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive.setDrivePowers(new PoseVelocity2d(
                      new Vector2d(
                             newForward,
                              -newRight
                      ),
                      -rotate
              ));
    }
}
