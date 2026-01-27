package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@TeleOp(name = "April Tag Limelight Testing", group = "robot")
public class AprilTagLimeLightTesting extends OpMode {

    private Limelight3A limelight;

    public HwRobot r = null;

    public static double yaw;


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        r = new HwRobot(telemetry, hardwareMap);
        limelight.pipelineSwitch(2);
        r.init();
        //to get heading, use variable = r.drive.localizer.getPose().heading.toDouble()

    }

    @Override
    public void start() {
        limelight.start();


    }

    @Override
    public void loop() {
        yaw = r.drive.localizer.getPose().heading.toDouble();
        limelight.updateRobotOrientation(yaw);
        LLResult llresult = limelight.getLatestResult();
        if(llresult != null && llresult.isValid()){
            Pose3D botPose = llresult.getBotpose_MT2();
            telemetry.addData("target X", llresult.getTx());
            telemetry.addData("target Y", llresult.getTy());
            telemetry.addData("target Area", llresult.getTa());
        }

    }
}
