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
    private DcMotor intakeMotor1 = null;
    private DcMotor intakeMotor2 = null;


    public static double INTAKEMOTOR_POWER_ON = 0.95;
    private final double INTAKEMOTOR_POWER_OFF = 0;

    public static double SLIDESERVO_EXTEND_POSITION = 0.215;

    //Constructor
    public Intake(HardwareMap hwmap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor1 = hwmap.get(DcMotor.class, "intakem1");
        intakeMotor2 = hwmap.get(DcMotor.class, "im2");



        //initial directions and positions
        intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);



        intakeMotorOff();

    }


    public void intakeMotorOff(){
        intakeMotor1.setPower(INTAKEMOTOR_POWER_OFF);
        intakeMotor2.setPower(INTAKEMOTOR_POWER_OFF);
    }

    public void intakeMotorOn(double power){
        double truePower;
        if(power < 0.3){
            truePower = 0.3;
        } else if (power > 0.95) {
            truePower = 0.95;
        }
        else{
            truePower = power;
        }
        intakeMotor1.setPower(truePower);
        intakeMotor2.setPower(truePower);
    }

    public void intakeMotorOn(){
        intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor1.setPower(INTAKEMOTOR_POWER_ON);
        intakeMotor2.setPower(INTAKEMOTOR_POWER_ON);
    }

    public void intakeMotorForward(){
        intakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor1.setPower(INTAKEMOTOR_POWER_ON);
        intakeMotor2.setPower(INTAKEMOTOR_POWER_ON);
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
