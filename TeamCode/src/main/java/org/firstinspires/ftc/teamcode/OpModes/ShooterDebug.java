package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwRobot;
@TeleOp(name = "Shooter Debug", group = "Concept")
public class ShooterDebug extends OpMode {
    public HwRobot r = null;

    @Override

    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        r = new HwRobot(telemetry, hardwareMap);
        r.init();

    }

    @Override
    public void init_loop(){

    }

    public void start(){



    }

    @Override
    public void loop() {
        r.outtake.launcherMotor1.setVelocity(2700*gamepad1.left_trigger);

        r.outtake.launcherMotor2.setVelocity(2700*gamepad1.right_trigger);

        telemetry.addData("LaunchMotorVelocity 1",r.outtake.launcherMotor1.getVelocity());

        telemetry.addData("LauncherMotorVelocity 2",r.outtake.launcherMotor2.getVelocity());

        //motor 1 velocity with the encoder cable plugged in is being read as motor 2 velocity

        //reverse shooter motor 2

        //replace encoder cable2
    }
}
