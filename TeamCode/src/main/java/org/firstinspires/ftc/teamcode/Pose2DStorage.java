package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pose2DStorage {
    public static Pose2d StordedPose = new Pose2d(0, 0,0);

    public void setNewPose(Pose2d currentPose){
        StordedPose = currentPose;
    }

}
