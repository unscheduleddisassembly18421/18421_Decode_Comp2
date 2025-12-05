package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class Outtake {

    Telemetry telemetry = null;

    //hardware
    private  DcMotorEx launcherMotor1 = null;
    private DcMotorEx launcherMotor2 = null;
    private DcMotor elavatorMotor = null;
    private Servo hoodServo1 = null;
    private Servo liftServo1 = null;
    private Servo liftServo2 = null;
    //private Servo hoodServo2 = null;

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
    public static double LIFTSERVO_LIFT_POSITION = 0.35;

    public static double newP = 0;
    public static double newI = 0;
    public static double newD = 0;


    //constructor
    public Outtake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        hoodServo1 = hwmap.get(Servo.class, "hs1 ");
        //hoodServo2 = hwmap.get(Servo.class, "hs2");
        launcherMotor1 = hwmap.get(DcMotorEx.class, "Lm1");
        launcherMotor2 = hwmap.get(DcMotorEx.class, "Lm2");
        elavatorMotor = hwmap.get(DcMotor.class, "em");
        liftServo1 = hwmap.get(Servo.class, "ls1");
        liftServo2 = hwmap.get(Servo.class, "lm2");

        hoodServo1.setDirection(Servo.Direction.FORWARD);
        //hoodServo2.setDirection(Servo.Direction.REVERSE);

        liftServo1.setDirection(Servo.Direction.FORWARD);
        liftServo2.setDirection(Servo.Direction.REVERSE);

        launcherMotor1.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor2.setDirection(DcMotor.Direction.FORWARD);
        elavatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo1.setPosition(HOODSERVO_START_POSITION);

        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elavatorMotor.setPower(0);
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

    public void elavatorMotorON(){
        elavatorMotor.setPower(ELAVATORMOTOR_POWER_ON);
    }

    public void elavatorMotorOff(){
        elavatorMotor.setPower(0);
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
        liftServo1.setPosition(0);
        liftServo2.setPosition(0);
    }

    public void liftServoLift(){
        liftServo1.setPosition(LIFTSERVO_LIFT_POSITION);
        liftServo2.setPosition(LIFTSERVO_LIFT_POSITION);
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


    public class TurnElavatorMotorOn implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elavatorMotorON();
            return false;
        }
    }
    public Action turnElavatorMotorOn(){
        return new TurnElavatorMotorOn();
    }

    public class TurnElavatorMotorOff implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            elavatorMotorOff();
            return false;
        }
    }
    public Action turnElavatorMotorOff(){
        return new TurnElavatorMotorOff();
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
