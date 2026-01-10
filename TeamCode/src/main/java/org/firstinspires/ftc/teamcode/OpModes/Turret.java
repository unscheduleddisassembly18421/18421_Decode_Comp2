package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret {
    Telemetry telemetry = null;
    double position;
    double targetPosition = 0;
    public static double kp = 0.005;
    public static double kd = 0;
    public static double ki = 0;

    public static double TOTAL_DEGREES = 360;

    public static double degreesPerRotation = 1./360;

    public static double CLOCK_DELAY = 2000;


    public static double intakeTolerance = 6;

    ServoPIDController rotatorPower = new ServoPIDController(kp, kd, ki);

    private final double DEGREES_PER_VOLT = 360 / 3.3;

    double ROTATOR_OFF = 0;

    private Servo rotatorServo = null;
    private Servo rotatorServo2 = null;


    private Servo leftLight = null;
    private Servo rightLight = null;

    public static double GREEN_COLOR = 0.5;
    public static double RED_COLOR = 0.28;
    public static double BLUE_COLOR = 0.62;

    private AnalogInput ai = null;


    public Turret(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        rotatorServo = hwmap.get(Servo.class, "rs");
        rotatorServo2 = hwmap.get(Servo.class, "rs2");
        ai = hwmap.get(AnalogInput.class, "ai");
//        position = getPosition();
        rotatorServo.setDirection(Servo.Direction.FORWARD);
        rotatorServo2.setDirection(Servo.Direction.FORWARD);


        rightLight = hwmap.get(Servo.class, "rightil");
        leftLight = hwmap.get(Servo.class, "leftil");

        leftLight.setPosition(0);
        rightLight.setPosition(0);

        rightLight.setDirection(Servo.Direction.FORWARD);
        leftLight.setDirection(Servo.Direction.FORWARD);


        //rotatorServo.setPosition(ROTATORSERVO_FIRST_POSITION);

    }

    public void init() {
        rotatorServo.setPosition(0.5);
        rotatorServo2.setPosition(0.5);
    }

    //color sensor stuff


    //PID servo stuff


    //indicator light stuff
    public void leftLightNewColor() {
        leftLight.setPosition(BLUE_COLOR);
    }

    public void leftLightRed() {
        leftLight.setPosition(RED_COLOR);
    }

    public void rightLightGreen() {
        rightLight.setPosition(GREEN_COLOR);
    }

    public void rightLightRed() {
        rightLight.setPosition(RED_COLOR);
    }

    public void setAngleRed(double angle){
        //rotatorServo.setDirection(Servo.Direction.FORWARD);
        //rotatorServo2.setDirection(Servo.Direction.FORWARD);
        double degrees = 1./TOTAL_DEGREES;
        double servoPosition = angle* degrees + 0.5; //fix degrees per Rotation
        rotatorServo.setPosition(servoPosition);
        rotatorServo2.setPosition(servoPosition);
        telemetry.addData("degrees per rotation", degrees);
    }

    public void setAngleBlue(double angle){
        //rotatorServo.setDirection(Servo.Direction.FORWARD);
        //rotatorServo2.setDirection(Servo.Direction.FORWARD);
        double ServoPosition = angle*degreesPerRotation + 0.5;
        rotatorServo.setPosition(ServoPosition);
        rotatorServo2.setPosition(ServoPosition);
    }

    public void startPosition(){
        rotatorServo.setPosition(0.5);
        rotatorServo2.setPosition(0.5);
    }

    public void setPostion(double angle){
        rotatorServo.setPosition(angle);
        rotatorServo2.setPosition(angle);
    }

    public class SetTurretStart implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            startPosition();
            return false;
        }
    }

    public Action setTurretStart(){
        return new SetTurretStart();
    }
}
