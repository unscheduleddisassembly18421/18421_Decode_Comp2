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
    private  DcMotorEx launcherMotor1 = null;//left one
    private DcMotorEx launcherMotor2 = null;
    private Servo hoodServo = null;
    private Servo ballBlockServo1 = null;
    private Servo ballBlockServo2 = null;
    //Positions
    public static double HOODSERVO_START_POSITION  = 0;
    public static double HOODSERVO_SHOOT_POSITION = 0.45;
    public static double LAUNCHERMOTOR_VELOCITY_ON_TELEOP = 1425;//max is around 2700
    public static double CLOSE_LAUNCHERMOTOR_VELOCITY_ON = 1100;//test
    public static double LAUNCHER_TOLERANCE = 0.995;
    public static double AUTO_LAUNCHERMOTOR_VELOCITY_ON = 1450;
    public static double AUTO_HOODSERVO_SHOOT = 0.5;
    public static double BALLBLOCKSERVO_BLOCK_POSITION = 0.1;

    public static double newP = 525;
    public static double newI = 40;//tune more. stole from Brennan
    public static double newD = 20;//TRY and UPLOAD
    public static double newF = 0;
    //tune f to set velocity to lowest speed
    //tune i first
    //then add in a little p


    //constructor
    public Outtake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        hoodServo = hwmap.get(Servo.class, "hs1 ");
        launcherMotor1 = hwmap.get(DcMotorEx.class, "Lm1");
        launcherMotor2 = hwmap.get(DcMotorEx.class, "Lm2");
        ballBlockServo1 = hwmap.get(Servo.class, "bs1");
        ballBlockServo2 = hwmap.get(Servo.class, "bs2");

        hoodServo.setDirection(Servo.Direction.FORWARD);

        ballBlockServo1.setDirection(Servo.Direction.FORWARD);
        ballBlockServo2.setDirection(Servo.Direction.REVERSE);


        hoodServo.setPosition(HOODSERVO_START_POSITION);
        //flywheel motor stuff

        launcherMotor1.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotor1.setVelocityPIDFCoefficients(newP, newI, newD, newF);
        launcherMotor2.setVelocityPIDFCoefficients(newP, newI, newD, newF);

    }

    public void init(){
        hoodServo.setPosition(0);
        ballBlockServoStart();
    }


    //commands
    public void activateShooterTeleop(){
        launcherMotor2.setVelocity(LAUNCHERMOTOR_VELOCITY_ON_TELEOP);
        launcherMotor1.setVelocity(LAUNCHERMOTOR_VELOCITY_ON_TELEOP);
        launcherMotor2.setVelocityPIDFCoefficients(newP, newI, newD, newF);
        launcherMotor1.setVelocityPIDFCoefficients(newP, newI, newD, newF);
    }

    public void flywheelOnInput(double inputVelocity){
        launcherMotor1.setVelocity(inputVelocity);
        launcherMotor2.setVelocity(inputVelocity);
    }


    public void launcherMotor1OnNear(){
        launcherMotor1.setVelocity(CLOSE_LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotor1Off(){
        launcherMotor1.setPower(0);
    }

    public void launcherMotor2OnNear(){
        launcherMotor2.setVelocity(CLOSE_LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void launcherMotor2Off(){
        launcherMotor2.setPower(0);
    }


    public void  hoodServoStart(){
        hoodServo.setPosition(0);
    }

    public void shootFar(){
        launcherMotor1.setVelocity(LAUNCHERMOTOR_VELOCITY_ON_TELEOP);
        launcherMotor2.setVelocity(LAUNCHERMOTOR_VELOCITY_ON_TELEOP);
    }

    public void hoodServoShoot(){
        hoodServo.setPosition(HOODSERVO_SHOOT_POSITION);
    }

    public void ballBlockServoStart(){
        ballBlockServo1.setPosition(0);
        ballBlockServo2.setPosition(0);
    }

    public void ballBlocKServoBlock(){
        ballBlockServo1.setPosition(BALLBLOCKSERVO_BLOCK_POSITION);
        ballBlockServo2.setPosition(BALLBLOCKSERVO_BLOCK_POSITION);
    }

    public boolean launchMotorsAtVelocity(){
        return (launcherMotor1.getVelocity() > LAUNCHER_TOLERANCE* LAUNCHERMOTOR_VELOCITY_ON_TELEOP) &&
                (launcherMotor2.getVelocity() > LAUNCHER_TOLERANCE* LAUNCHERMOTOR_VELOCITY_ON_TELEOP);
    }

    public boolean autoLaunchMotorsAtVelocity(){
        return (launcherMotor1.getVelocity() > LAUNCHER_TOLERANCE*AUTO_LAUNCHERMOTOR_VELOCITY_ON) &&
                (launcherMotor2.getVelocity() > LAUNCHER_TOLERANCE*AUTO_LAUNCHERMOTOR_VELOCITY_ON );
    }

    public boolean launcherMotorsAtVelocityNear(){
        return (launcherMotor1.getVelocity() > LAUNCHER_TOLERANCE*CLOSE_LAUNCHERMOTOR_VELOCITY_ON) &&
                (launcherMotor2.getVelocity() > LAUNCHER_TOLERANCE*CLOSE_LAUNCHERMOTOR_VELOCITY_ON);
    }

    public void hoodServoRelative(double distance){
        double trueAngle;
        double angle = (0.00741 * distance) -0.16667;
        if(angle > 0.5){
            trueAngle = 0.45;
        }
        else{
            trueAngle = angle;
        }
        hoodServo.setPosition(trueAngle);
    }

    public void activateShooterRelative(double robotdistance){
        double velocity = -(0.0248016 * (robotdistance * robotdistance)) + (8.39286 * robotdistance) + 637.85714;
        launcherMotor1.setVelocity(velocity);
        launcherMotor2.setVelocity(velocity);
    }

    public double getVelocity1(){
        return (launcherMotor1.getVelocity());
    }

    public double getVelocity2(){
        return -(launcherMotor2.getVelocity());
    }

    public void deactivateShooter(){
        launcherMotor1Off();
        launcherMotor2Off();
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
            hoodServo.setPosition(AUTO_HOODSERVO_SHOOT);
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
            hoodServoShoot();
            return false;
        }
    }
    public Action openHoodServoNear(){
        return new OpenHoodServoNear();
    }

    public class BallServoBlock implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ballBlocKServoBlock();
            return false;
        }
    }
    public Action ballBlockServo(){
        return new BallServoBlock();
    }

    public class BallServoOpen implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ballBlockServoStart();
            return false;
        }
    }
    public Action ballServoOpen(){
        return new BallServoOpen();
    }



}
