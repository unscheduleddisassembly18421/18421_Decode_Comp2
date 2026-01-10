package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpModes.Intake;
import org.firstinspires.ftc.teamcode.OpModes.Outtake;
import org.firstinspires.ftc.teamcode.OpModes.Turret;

public class HwRobot {
    public MecanumDrive drive;
    public Intake intake = null;
    public Outtake outtake = null;
    public Turret turret = null;
    Telemetry telemetry = null;
    HardwareMap hardwareMap = null;
    Pose2d BlueWallRight = new Pose2d(0,0,0);

    Pose2d blueGoalPose = new Pose2d(-65,-61,0);//find real pose
    Pose2d redGoalPose = new Pose2d(-65,61,0);//find real pose

    public HwRobot(Telemetry t, HardwareMap hwm){
        hardwareMap = hwm;
        telemetry = t;
    }

    public void init(){
        drive = new MecanumDrive(hardwareMap, BlueWallRight);
        intake = new Intake(hardwareMap, telemetry);
        outtake = new Outtake(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.init();
        outtake.init();
    }


    public double[] autoTurretTracking(Pose2d robotPose, Pose2d goalPose){
        double poseRobotX = robotPose.position.x;
        double poseRobotY = robotPose.position.y;
        double robotYaw = robotPose.heading.toDouble();
        double poseGoalX = goalPose.position.x;
        double poseGoalY = goalPose.position.y;
        double xDValue = poseRobotX - poseGoalX;
        double yDValue = poseRobotY - poseGoalY;
        double d = Math.sqrt((xDValue * xDValue) + (yDValue * yDValue));
        double fieldAngle = Math.atan2(yDValue, xDValue);
        double goalAngle = robotYaw + fieldAngle;
        double goalAngleDegrees = Math.toDegrees(goalAngle);
        telemetry.addData("distance", d);
        telemetry.addData("target angle", goalAngleDegrees);
        double[] aimToGoal = new double[2];
        aimToGoal[0] = d;
        aimToGoal[1] = goalAngleDegrees;
        return aimToGoal;
    }

    public void turnTurret(double angle){
        //DO SOMETHING
    }

    public Action activateShooter(){
        return outtake.activateShooter();
    }

    public Action activateShooterNear(){
        return outtake.activateShooterNear();
    }

    public Action turnOffShooter(){
        return outtake.turnOffShooter();
    }

    public Action checkShooterVelocity(){
        return outtake.checkShooterVelocity();
    }

    public Action checkShooterVelocityNear(){
        return outtake.checkShooterVelocityNear();
    }

    public Action openHoodServo(){
        return outtake.openHoodServoFar();
    }

    public Action closeHoodServo(){
        return outtake.closeHoodServo();
    }

    public Action openHoodServoNear(){
       return outtake.openHoodServoNear();
    }


    public Action turnOnIntake(){
        return intake.turnOnIntake();
    }

    public Action turnOffIntake(){
        return intake.turnOffIntake();
    }

    public Action setTurretStart(){
        return turret.setTurretStart();
    }



    public Action reverseIntake(){
        return intake.reverseIntake();
    }

    public Action ballBlockServoBlock(){
        return outtake.ballBlockServo();
    }

    public Action ballBlockServoOpen(){
        return outtake.ballServoOpen();
    }

    public void aimTurretRed(){
        TurretAim turretAim = new TurretAim(drive.localizer.getPose(),redGoalPose);
        //shooterRelativeVelocityRed();
        double angle = (turretAim.redAngle * turret.degreesPerRotation) + 0.5;
        if(angle > 1 || angle < 0){
            turret.startPosition();
        }
        else{
            turret.setAngleRed(turretAim.redAngle);
        }
        telemetry.addData("target angle", turretAim.redAngle);
        telemetry.addData("servo position", angle);

    }

    public void aimTurretBlue(){
        TurretAim turretAim = new TurretAim(drive.localizer.getPose(), blueGoalPose);
        //shooterRelativeVelocityBlue();
        double angle = (turretAim.redAngle * turret.degreesPerRotation) + 0.5;
        if(angle > 1|| angle < 0){
            turret.startPosition();
        }
        else{
            turret.setAngleBlue(turretAim.blueAngle);
        }
        telemetry.addData("target angle", turretAim.blueAngle);
        telemetry.addData("servo position", angle);

    }

    public void shooterRelativeVelocityBlue(){
        TurretAim turretAim = new TurretAim(drive.localizer.getPose(), blueGoalPose);
        double velocity = turretAim.distance;//write function for this here
        outtake.flywheelOnInput(velocity);
    }

    public void shooterRelativeVelocityRed(){
        TurretAim turretAim = new TurretAim(drive.localizer.getPose(), redGoalPose);
        double velocity = turretAim.distance;//write function here for this
        outtake.flywheelOnInput(velocity);
    }

    public class TurretAim {
        public double distance;
        public double blueAngle;
        public double redAngle ;

        public TurretAim(Pose2d robotPose, Pose2d goalPose) {
            double poseRobotX = robotPose.position.x;
            double poseRobotY = robotPose.position.y;
            double robotYaw = robotPose.heading.toDouble();
            double poseGoalX = goalPose.position.x;
            double poseGoalY = goalPose.position.y;
            double xDValue = poseGoalX - poseRobotX;
            double yDValue = poseGoalY - poseRobotY;
            double d = Math.sqrt((xDValue * xDValue) + (yDValue * yDValue));
            double fieldAngle = Math.atan2(yDValue, xDValue);
            blueAngle = Math.toDegrees((robotYaw - fieldAngle));
            redAngle = Math.toDegrees((robotYaw - fieldAngle));
            distance = d;

            //build in red and blue goal poses, such that it only uses the robot pose to calculate its angle
        }
    }
            //right front is in EH 0 named rf
            //right back is in EH 1 named rb
            //left back is in CH 1 named lb
            //left front is in CH 0 named lf
            //hood servo is in CH 1 named hs1
            //left indicator light is in CH 3 named leftil
            //right indicator light is in CH 4 named rightil
            //pinpoint is in I2C bus 0 CH named pinpoint
            //rotator servo is in CH 0 named rs
            //elavator motor is in EH 2 named em
            //intake motor is in EH 3 named intakem
            //launcher motor 1 is in CH 3 named Lm1
            //launcher motor 2 is in CH 2 named Lm2
            //analog input is in CH analog input 0 named ai
            //right color sensor is in EH I2C 0 named rightcs
            //left color sensor is in CH I2C 2  named leftcs
            //intake color sensor is in CH I2C 1 named ics
            //lift servo 1 is in CH servo 5, named ls1
            //lift servo 2 id in EH servo 0, named ls2


            //COMP 2 HW MAp
            //right front is CH 3
            //right back is in CH 2
            //left front is in CH 0
            //left back is in CH 1

}
