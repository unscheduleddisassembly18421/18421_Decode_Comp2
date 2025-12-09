package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class Outtake {

    Telemetry telemetry = null;

    //hardware
    private  DcMotorEx launcherMotor1 = null;
    private DcMotorEx launcherMotor2 = null;
    private Servo hoodServo1 = null;
    private Servo ballBlockServo1 = null;
    private Servo ballBlockServo2 = null;
    //Positions
    public static double HOODSERVO_START_POSITION  = 0;
    public static double HOODSERVO_SHOOT_POSITION = 0.3;
    public static double HOODSERVO_CLOSE_SHOOT_POSITION = 0.4;
    public static double FAR_LAUNCHERMOTOR_VELOCITY_ON = 1950;//max is around 2700
    public static double CLOSE_LAUNCHERMOTOR_VELOCITY_ON = 1675;//test
    public static double ELAVATORMOTOR_POWER_ON = 1;
    public static double LAUNCHER_TOLERANCE = 0.995;
    public static double AUTO_LAUNCHERMOTOR_VELOCITY_ON = 1980;
    public static double AUTO_HOODSERVO_SHOOT = 0.525;
    public static double BALLBLOCKSERVO_LIFT_POSITION = 0.35;

    public static double newP = 0;
    public static double newI = 0;
    public static double newD = 0;
    public static double newF = 0;
    //tune f to set velocity to lowest speed
    //tune i first
    //then add in a little p


    //constructor
    public Outtake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        hoodServo1 = hwmap.get(Servo.class, "hs1 ");
        //hoodServo2 = hwmap.get(Servo.class, "hs2");
        launcherMotor1 = hwmap.get(DcMotorEx.class, "Lm1");
        launcherMotor2 = hwmap.get(DcMotorEx.class, "Lm2");
        ballBlockServo1 = hwmap.get(Servo.class, "bs1");
        ballBlockServo2 = hwmap.get(Servo.class, "bs2");

        hoodServo1.setDirection(Servo.Direction.FORWARD);

        ballBlockServo1.setDirection(Servo.Direction.FORWARD);
        ballBlockServo2.setDirection(Servo.Direction.REVERSE);

        launcherMotor1.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor2.setDirection(DcMotor.Direction.FORWARD);

        hoodServo1.setPosition(HOODSERVO_START_POSITION);

        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotor1.setVelocityPIDFCoefficients(newP, newI, newD, newF);
        launcherMotor2.setVelocityPIDFCoefficients(newP, newI, newD, newF);

    }

    public void init(){
        hoodServo1.setPosition(HOODSERVO_START_POSITION);
        liftServoStart();
    }


    //commands
    public void launcherMotor1OnFar(){
        launcherMotor1.setVelocity(FAR_LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotor1OnNear(){
        launcherMotor1.setVelocity(CLOSE_LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotor1Off(){
        launcherMotor1.setVelocity(0);
    }

    public void  launcherMotor2OnFar(){
        launcherMotor2.setVelocity(FAR_LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotor2OnNear(){
        launcherMotor2.setVelocity(CLOSE_LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotor2Off(){
        launcherMotor2.setVelocity(0);
    }


    public void  hoodServoStart(){
        hoodServo1.setPosition(HOODSERVO_START_POSITION);
    }

    public void hoodServoShootNear(){
        hoodServo1.setPosition(HOODSERVO_CLOSE_SHOOT_POSITION);
    }


    public void  hoodServoShootFar(){
        hoodServo1.setPosition(HOODSERVO_SHOOT_POSITION);
    }

    public void liftServoStart(){
        ballBlockServo1.setPosition(0);
        ballBlockServo2.setPosition(0);
    }

    public void liftServoLift(){
        ballBlockServo1.setPosition(BALLBLOCKSERVO_LIFT_POSITION);
        ballBlockServo2.setPosition(BALLBLOCKSERVO_LIFT_POSITION);
    }

    public boolean launchMotorsAtVelocity(){
        return (launcherMotor1.getVelocity() > LAUNCHER_TOLERANCE*FAR_LAUNCHERMOTOR_VELOCITY_ON) &&
                (launcherMotor2.getVelocity() > LAUNCHER_TOLERANCE*FAR_LAUNCHERMOTOR_VELOCITY_ON );
    }

    public boolean autoLaunchMotorsAtVelocity(){
        return (launcherMotor1.getVelocity() > LAUNCHER_TOLERANCE*AUTO_LAUNCHERMOTOR_VELOCITY_ON) &&
                (launcherMotor2.getVelocity() > LAUNCHER_TOLERANCE*AUTO_LAUNCHERMOTOR_VELOCITY_ON );
    }

    public boolean launcherMotorsAtVelocityNear(){
        return (launcherMotor1.getVelocity() > LAUNCHER_TOLERANCE*CLOSE_LAUNCHERMOTOR_VELOCITY_ON) &&
                (launcherMotor2.getVelocity() > LAUNCHER_TOLERANCE*CLOSE_LAUNCHERMOTOR_VELOCITY_ON);
    }

    public double getVelocity1(){
        return (launcherMotor1.getVelocity());
    }

    public double getVelocity2(){
        return (launcherMotor2.getVelocity());
    }

    //actions

    public class ActivateShooter implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherMotor1.setVelocity(AUTO_LAUNCHERMOTOR_VELOCITY_ON);
            launcherMotor2.setVelocity(AUTO_LAUNCHERMOTOR_VELOCITY_ON);
            return false; //if the return is false, then the action ends!  If true, it continues.
        }
    }

    public Action activateShooter(){
        return new ActivateShooter();
    }

    public class ActivateShooterNear implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherMotor1OnNear();
            launcherMotor2OnNear();
            return false;
        }
    }

    public Action activateShooterNear(){
        return new ActivateShooterNear();
    }

    public class TurnOffShooter implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherMotor1Off();
            launcherMotor2Off();
            return false;
        }
    }
    public Action turnOffShooter(){
        return new TurnOffShooter();
    }


    public class CheckShooterVelocity implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !autoLaunchMotorsAtVelocity();
        }
    }
    public Action checkShooterVelocity(){
        return new CheckShooterVelocity();
    }

    public class CheckShooterVelocityNear implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !launcherMotorsAtVelocityNear();
        }
    }
    public Action checkShooterVelocityNear(){
        return new CheckShooterVelocityNear();
    }



    public class OpenHoodServoFar implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hoodServo1.setPosition(AUTO_HOODSERVO_SHOOT);
            return false;
        }
    }
    public Action openHoodServoFar(){
        return new OpenHoodServoFar();
    }

    public class CloseHoodServo implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hoodServoStart();
            return false;
        }
    }

    public Action closeHoodServo(){
        return new CloseHoodServo();
    }

    public class OpenHoodServoNear implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            hoodServoShootNear();
            return false;
        }
    }
    public Action openHoodServoNear(){
        return new OpenHoodServoNear();
    }



}
