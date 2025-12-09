package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Config
public class Turret {
    Telemetry telemetry = null;
    double position;
    double targetPosition = 0;
    public static double kp = 0.005;
    public static double kd = 0;
    public static double ki = 0;

    public static double CLOCK_DELAY = 2000;


    public static double intakeTolerance = 6;

    ServoPIDController rotatorPower = new ServoPIDController(kp,kd,ki);

    private final double DEGREES_PER_VOLT = 360/3.3;

    double ROTATOR_OFF = 0;

    private CRServo rotatorServo = null;
    private NormalizedColorSensor leftSlideColorSensor = null;
    private NormalizedColorSensor rightSlideColorSensor = null;
    private NormalizedColorSensor rightOtherColorSensor = null;
    private NormalizedColorSensor leftOtherColorSensor = null;

    private Servo leftLight = null;
    private Servo rightLight = null;

    public static double GREEN_COLOR = 0.5;
    public static double RED_COLOR = 0.28;
    public static double BLUE_COLOR = 0.62;

    private AnalogInput ai = null;

    float slideCSGain = 4;
    float otherCSGain = 2;
    float rightGain = 2;

    float[] rightSlideColorHSV = new float[3];
    float[] leftSlideColorHSV = new float[3];
    float[] leftOtherColorHSV = new float[3];
    float[] rightOtherColorHSV = new float[3];


    public Turret(HardwareMap hwmap, Telemetry telemetry){
        this.telemetry = telemetry;

        rotatorServo = hwmap.get(CRServo.class, "rs");
        ai = hwmap.get(AnalogInput.class,"ai");
        position = getPosition();
        rotatorServo.setDirection(CRServo.Direction.REVERSE);

        leftSlideColorSensor = hwmap.get(NormalizedColorSensor.class, "lscs");
        rightSlideColorSensor = hwmap.get(NormalizedColorSensor.class, "rscs");
        leftOtherColorSensor = hwmap.get(NormalizedColorSensor.class, "locs");
        rightOtherColorSensor = hwmap.get(NormalizedColorSensor.class, "rocs");

        rightLight = hwmap.get(Servo.class, "rightil");
        leftLight = hwmap.get(Servo.class, "leftil");

        leftLight.setPosition(0);
        rightLight.setPosition(0);

        rightLight.setDirection(Servo.Direction.FORWARD);
        leftLight.setDirection(Servo.Direction.FORWARD);

       leftSlideColorSensor.setGain(slideCSGain);
        rightSlideColorSensor.setGain(slideCSGain);
        leftOtherColorSensor.setGain(otherCSGain);
        rightOtherColorSensor.setGain(otherCSGain);

        //rotatorServo.setPosition(ROTATORSERVO_FIRST_POSITION);

    }

    public void init(){
        setPower(ROTATOR_OFF);
    }

    //color sensor stuff
    public void readColorSensors(){
        NormalizedRGBA rightSlideColor = rightSlideColorSensor.getNormalizedColors();
        NormalizedRGBA leftSlideColor = leftSlideColorSensor.getNormalizedColors();
        NormalizedRGBA rightOtherColor = rightOtherColorSensor.getNormalizedColors();
        NormalizedRGBA leftOtherColor = leftOtherColorSensor.getNormalizedColors();
        Color.colorToHSV(rightSlideColor.toColor(), rightSlideColorHSV);
        Color.colorToHSV(leftSlideColor.toColor(), leftSlideColorHSV);
        Color.colorToHSV(leftOtherColor.toColor(), leftOtherColorHSV);
        Color.colorToHSV(rightSlideColor.toColor(), rightOtherColorHSV);
        telemetry.addData("HSV right slide CS", Arrays.toString(rightSlideColorHSV));
        telemetry.addData("HSV left slide CS", Arrays.toString(leftSlideColorHSV));
        telemetry.addData("HSV left other CS", Arrays.toString(leftOtherColorHSV));
        telemetry.addData("HSV right Colro HSV", Arrays.toString(rightOtherColorHSV));
    }


    //PID servo stuff
    double getPosition(){
        return ai.getVoltage()*DEGREES_PER_VOLT;
    }

    public void setPower(double power){
        rotatorServo.setPower(power);
    }

    public void setPosition(double angle){
        targetPosition = angle;
    }

    public void update(){
        rotatorPower.setPIDConstants(kp,kd,ki);
        double power = rotatorPower.calculate(targetPosition,getPosition());
        setPower(power);
        telemetry.addData("Rotator Position",getPosition());
        telemetry.addData("Rotator Power",power);
    }

    //indicator light stuff
    public void leftLightNewColor(){
        leftLight.setPosition(BLUE_COLOR);
    }

    public void leftLightRed(){
        leftLight.setPosition(RED_COLOR);
    }

    public void rightLightGreen(){
        rightLight.setPosition(GREEN_COLOR);
    }

    public void rightLightRed(){
        rightLight.setPosition(RED_COLOR);
    }


    //actions

    public class UpdateRotator implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return true;
        }
    }

    public Action updateRotator(){
        return new UpdateRotator();
    }
}
