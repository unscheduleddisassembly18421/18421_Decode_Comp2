package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    Telemetry telemetry = null;

    //Hardware
    private DcMotor intakeMotor = null;

    private Servo slideServo1 = null;
    private Servo slideServo2 = null;

    public static double INTAKEMOTOR_POWER_ON = 0.95;
    private static double INTAKEMOTOR_POWER_OFF = 0;

    public static double SLIDESERVO_EXTEND_POSITION = 0.5;

    //Constructor
    public Intake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hwmap.get(DcMotor.class, "intakem");

        slideServo1 = hwmap.get(Servo.class, "slide1");
        slideServo2 = hwmap.get(Servo.class, "slide2");

        //initial directions and positions
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        slideServo1.setDirection(Servo.Direction.REVERSE);
        slideServo2.setDirection(Servo.Direction.FORWARD);

        intakeMotorOff();

        slideServostart();
    }


    public void intakeMotorOff(){
        intakeMotor.setPower(INTAKEMOTOR_POWER_OFF);
    }

    public void intakeMotorOn(){
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setPower(INTAKEMOTOR_POWER_ON);
    }

    public void intakeMotorForward(){
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(INTAKEMOTOR_POWER_ON);
    }

    public void slideServostart(){
        slideServo1.setPosition(0);
        slideServo2.setPosition(0);
    }

    public void slideServoExtend(){
        slideServo1.setPosition(SLIDESERVO_EXTEND_POSITION);
        slideServo2.setPosition(SLIDESERVO_EXTEND_POSITION);
    }


    public class TurnOnIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotorOn();
            return false;
        }
    }

    public Action turnOnIntake(){
        return new TurnOnIntake();
    }

    public class TurnOffIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotorOff();
            return false;
        }
    }

    public Action turnOffIntake(){
        return new TurnOffIntake();
    }

    public class ReverseIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotorForward();
            return false;
        }
    }

    public Action reverseIntake(){
        return new ReverseIntake();
    }


}
