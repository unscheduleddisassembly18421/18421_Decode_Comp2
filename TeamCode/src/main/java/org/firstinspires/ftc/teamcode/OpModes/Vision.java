package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
public class Vision {
    private Limelight3A limelight;
    Telemetry telemetry = null;

    public static double targetX;
    public static double targetY;
    public static double targetAera;

    public static double distance;

    public static Pose2d botPose;

    public Vision(HardwareMap Hwmap, Telemetry telemetry){
        this.telemetry = telemetry;
        limelight = Hwmap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();
    }

    public void updateLimelight(double yaw){
        limelight.updateRobotOrientation(yaw);
        LLResult llresult = limelight.getLatestResult();
        if(llresult != null && llresult.isValid()){
            List<LLResultTypes.FiducialResult> fiducialResults = llresult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == 24 || fr.getFiducialId() == 20){
                    targetX = fr.getTargetPoseRobotSpace().getPosition().x;
                    targetY = fr.getTargetPoseRobotSpace().getPosition().y;
                    distance = Math.sqrt((targetX*targetX) + (targetY*targetY));
                }
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        botPose = new Pose2d(llresult.getBotpose().getPosition().x, llresult.getBotpose().getPosition().y, llresult.getBotpose().getOrientation().getYaw());


        }

    }




}
