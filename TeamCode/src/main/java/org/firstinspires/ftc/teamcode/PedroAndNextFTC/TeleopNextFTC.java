package org.firstinspires.ftc.teamcode.PedroAndNextFTC;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import dev.nextftc.ftc.NextFTCOpMode;

@Config
@TeleOp(name = "DriverControl NextFTC", group = "concept")
public class TeleopNextFTC extends NextFTCOpMode {
    MecanumDrive drive = null;
    Pose2d startPose = new Pose2d(0,0,0);
    Telemetry telemetry = null;
    public enum targetGoal{
        BLUE, RED
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, startPose);

    }
    @Override
    public void onWaitForStart() {
    }
    @Override
    public void onStartButtonPressed() {

    }
    @Override
    public void onUpdate() {

    }
    @Override
    public void onStop() {

    }
}
