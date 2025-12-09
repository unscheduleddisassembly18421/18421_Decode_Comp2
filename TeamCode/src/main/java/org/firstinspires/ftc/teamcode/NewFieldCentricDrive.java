package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Field Centric Drive:Use", group = "Concept")
public class NewFieldCentricDrive extends OpMode {
    public HwRobot r = null;

    Gamepad g1 = new Gamepad();
    Gamepad previousG1 = new Gamepad();
    public double yaw;
    double headingLockValue = 0;

    boolean headingLock;
    public static double error;
    public static double kp = 0;
    public double stickSensitivity = 0.05;

    @Override
    public void init() {
        r = new HwRobot(telemetry, hardwareMap);
        r.init();

    }

    @Override
    public void loop() {
        previousG1.copy(g1);
        g1.copy(gamepad1);
        yaw = r.drive.localizer.getPose().heading.toDouble(); //maybe in radians

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


        driveFieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x+(error*kp),
                yaw
        );

        telemetry.addData("heading lock", headingLock);
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed
        r.drive.leftBack.setPower(maxSpeed * (frontLeftPower / maxPower));
        r.drive.rightBack.setPower(maxSpeed * (frontRightPower / maxPower));
        r.drive.rightFront.setPower(maxSpeed * (backLeftPower / maxPower));
        r.drive.leftFront.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void driveFieldCentric(double forward, double right, double rotate, double newyaw){
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta -
                newyaw);

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);

    }
}
