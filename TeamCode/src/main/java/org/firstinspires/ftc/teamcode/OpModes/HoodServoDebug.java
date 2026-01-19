package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwRobot;
@Config
@TeleOp(name = "HoodServo Debug", group = "robot")

public class HoodServoDebug extends OpMode {
    public HwRobot r = null;
    @Override
    public void init() {
        r = new HwRobot(telemetry, hardwareMap);
        r.init();
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            r.outtake.hoodServoShoot();
        }

        if(gamepad1.x){
            r.outtake.hoodServoStart();
        }
    }
}
