package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
@Disabled

public class AprilTagLimeLightTesting extends OpMode {

    private Limelight3A limelight;

    public HwRobot r = null;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        r = new HwRobot(telemetry, hardwareMap);
        r.init();
        //to get heading, use variable = r.drive.localizer.getPose().heading.toDouble()


    }

    @Override
    public void start() {
        limelight.start();


    }

    @Override
    public void loop() {


    }
}
