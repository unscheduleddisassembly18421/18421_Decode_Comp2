package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class AprilTagLimeLightTesting extends OpMode {

    private Limelight3A limelight;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


    }

    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {


    }
}
