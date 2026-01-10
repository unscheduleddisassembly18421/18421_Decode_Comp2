

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.INTAKE_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.INTAKE_OFF_DELAY;
import static org.firstinspires.ftc.teamcode.Variables.firstAngle;
import static org.firstinspires.ftc.teamcode.Variables.firstShootingAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondAngle;
import static org.firstinspires.ftc.teamcode.Variables.secondShootingAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdAngle;
import static org.firstinspires.ftc.teamcode.Variables.thirdShootingAngle;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.DriverControl;

@Config
@Autonomous(name= "Automonous")
public class Automonous extends LinearOpMode {
    public enum AutoSelector {RED_FAR, RED_NEAR, BLUE_FAR, BLUE_NEAR}
    public AutoSelector autoSelector = AutoSelector.RED_FAR;
    public HwRobot r = null;
    public static double SHOOTING_DELAY = 0.45;
    public static double SELECTOR_DELAY_TIME = 0.4;

    public static double NEAR_DELAY = 10000;
    public static double MIDDLE_DELAY = 16500;
    public static double FAR_DELAY = 25000;

    public enum IntakeState {
        READY, INTAKE1, INTAKE2, INTAKE3, FULL, FIRING
    }

    IntakeState intakeState = IntakeState.READY;
    public ElapsedTime intakeClock = new ElapsedTime();

            private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d beginPose = new Pose2d(0, 0, 0);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        r = new HwRobot(telemetry, hardwareMap);
        r.init();


        while (opModeInInit()) {

            double time = runtime.seconds();
            telemetry.addData(" AUTO SELECTED", autoSelector);
            telemetry.addLine("D-Pad Up for Red Far");
            telemetry.addLine("D-Pad Right for Red Near");
            telemetry.addLine("D-Pad Down for Blue Far");
            telemetry.addLine("D-Pad Left for Blue Near");
            telemetry.update();

            if (gamepad1.dpad_up) {

                autoSelector = AutoSelector.RED_FAR;

            } else if (gamepad1.dpad_right) {

                autoSelector = AutoSelector.RED_NEAR;

            } else if (gamepad1.dpad_down) {

                autoSelector = AutoSelector.BLUE_FAR;

            } else if (gamepad1.dpad_left) {

                autoSelector = AutoSelector.BLUE_NEAR;
            }


        }

        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-63, 12, Math.toRadians(0));

        Pose2d blueStartFar = new Pose2d(63, -12, Math.toRadians(180));

        Pose2d blueStartNear = new Pose2d(-63, -12, Math.toRadians(0));

        if (autoSelector == AutoSelector.RED_FAR) {
            r.drive.localizer.setPose(redStartFar);
        } else if (autoSelector == AutoSelector.RED_NEAR) {
            r.drive.localizer.setPose(redStartNear);
        } else if (autoSelector == AutoSelector.BLUE_FAR) {
            r.drive.localizer.setPose(blueStartFar);
        } else {
            r.drive.localizer.setPose(blueStartNear);
        }


        //Make the trajectories here




        // RED FAR
        TrajectoryActionBuilder redFarMoveToShootingPosition = r.drive.actionBuilder(redStartFar)
                .lineToX(56)
                .turnTo(Math.toRadians(156))
                .endTrajectory();

        TrajectoryActionBuilder redFarFirstPath = redFarMoveToShootingPosition.fresh()//firstPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(36, 30,Math.toRadians(90)),
                        Math.toRadians(90))
                .lineToY(55,  new TranslationalVelConstraint(20))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(55, 15,Math.toRadians(158)),Math.toRadians(0))
                .endTrajectory();


        TrajectoryActionBuilder redFarSecondPath = redFarFirstPath.fresh()//secondPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13,30, Math.toRadians(90)), Math.toRadians(90))//13,35,90
                .lineToY(52, new TranslationalVelConstraint(15))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(155)),Math.toRadians(0)
                )
                .endTrajectory();


        TrajectoryActionBuilder redFarThirdPath = redFarSecondPath.fresh()//thirdPathFarRed
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-10, 30,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(55, new TranslationalVelConstraint(25))
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(50, 12,Math.toRadians(155)),
                        Math.toRadians(-30), new TranslationalVelConstraint(55))
                .endTrajectory();

        TrajectoryActionBuilder redFarThirdPathEnd = redFarThirdPath.fresh()
                .lineToX(35)
                .endTrajectory();


        //RED NEAR
        TrajectoryActionBuilder redNearFirstPath = r.drive.actionBuilder(redStartNear)//firstPathNearRed
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(18)
                .turnTo(Math.toRadians(145))
                .endTrajectory();

        TrajectoryActionBuilder redNearSecondPath = redNearFirstPath.fresh()//secondPathNearRed
                .strafeToLinearHeading(new Vector2d(12, 12),Math.toRadians(90))
//                    new TranslationalVelConstraint(40)
                .lineToY(56)
                .lineToY(12)
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(145))
                .endTrajectory() ;

        TrajectoryActionBuilder redNearThirdPath = redNearSecondPath.fresh()//thirdPathNearRed
                .strafeToLinearHeading(new Vector2d(36, 12), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(12)
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(135))
                .endTrajectory();

        //BLUE FAR
        TrajectoryActionBuilder blueFarMoveToShootingPosition = r.drive.actionBuilder(blueStartFar)//blueFarMoveToShootingPosition
                .lineToX(56)
                .turn(Math.toRadians(20))
                .endTrajectory();

        TrajectoryActionBuilder blueFarFirstPath = r.drive.actionBuilder(blueStartFar)//firstPathFarBlue
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(38, -35,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-55, new TranslationalVelConstraint(25))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55, -12,Math.toRadians(205)),Math.toRadians(0))
                .endTrajectory();

        TrajectoryActionBuilder blueFarSecondPath = blueFarFirstPath.fresh()//secondPathFarBlue
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(13, -35,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-50, new TranslationalVelConstraint(12))
                .splineToLinearHeading(new Pose2d(53, -12,Math.toRadians(205)),Math.toRadians(0))
                .endTrajectory();

        TrajectoryActionBuilder blueFarThirdPath = blueFarSecondPath.fresh()//thirdPathFarBlue
                .splineToLinearHeading(new Pose2d(-10, -38,Math.toRadians(270)),Math.toRadians(270))
                .lineToY(-53, new TranslationalVelConstraint(25))
                .splineToLinearHeading(new Pose2d(55, -15,Math.toRadians(200)),
                        Math.toRadians(-40), new TranslationalVelConstraint(55))
                .endTrajectory();
        TrajectoryActionBuilder blueFarThirdPathEnd = blueFarThirdPath.fresh()//thirdPathFarBlueEnd
                .lineToX(35)
                .endTrajectory();


        //BLUE NEAR
        TrajectoryActionBuilder blueNearFirstPath = r.drive.actionBuilder(blueStartNear)//firstPathNearBlue
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-18)
                .turnTo(Math.toRadians(220))
                .endTrajectory();

        TrajectoryActionBuilder blueNearSecondPath = blueNearFirstPath.fresh()//secondPathNearBlue
                .strafeToLinearHeading(new Vector2d(12, -12),Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-12)
                .strafeToLinearHeading(new Vector2d(-12,-12),Math.toRadians(270))
                .turnTo(Math.toRadians(220))
                .endTrajectory();

        TrajectoryActionBuilder blueNearThirdPath = blueNearSecondPath.fresh()//thirdPathNearBlue
                .strafeToLinearHeading(new Vector2d(36, -12), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-56)
                .lineToY(-12)
                .setTangent(Math.toRadians(220))
                .strafeToLinearHeading(new Vector2d(-12, -12), Math.toRadians(220))
                .endTrajectory();

        //build trajectories


        //switch (intakeState){
           // case READY:
                //r.rotator.leftLightRed();
                //r.rotator.rightLightRed();
                //r.rotator.setPosition(firstAngle);
                //r.intake.intakeMotorOn();
                //intakeState = IntakeState.INTAKE1;

                //break;

            //case INTAKE1:
               // if(r.rotator.detectedBall()){
                    //intakeState = IntakeState.INTAKE2;
                    //intakeClock.reset();
                //}
                //break;
           // case INTAKE2:
               // r.rotator.setPosition(secondAngle);
    //            if(intakeClock.milliseconds() > INTAKE_DELAY && r.rotator.detectedBall()){
      //              intakeClock.reset();
        //            intakeState = IntakeState.INTAKE3;
          //      }
            //    break;

     //       case INTAKE3:
     //           r.rotator.setPosition(thirdAngle);
     //           if(intakeClock.milliseconds() > INTAKE_DELAY && r.rotator.detectedBall()){
     //               intakeClock.reset();
     //               intakeState = IntakeState.FULL;
     //           }
     //           break;

     //       case FULL:
     //           if(intakeClock.milliseconds() > INTAKE_OFF_DELAY) {
     //               r.intake.intakeMotorOff();
      //              r.rotator.leftLightGreen();
     //               r.rotator.rightLightGreen();
     //               intakeState = IntakeState.FIRING;
     //           }

       //         break;
        //}

        //build trajectories
        //Action *NameOfPath* = nameOfPath.build();


        //RED FAR
        Action RedFarGoToShootingPosition = redFarMoveToShootingPosition.build();
        Action RedFarMoveToShootingFirstPath = redFarFirstPath.build();
        Action RedFarMoveToShootingSecondPath = redFarSecondPath.build();
        Action RedFarMoveToShootingThirdPath = redFarThirdPath.build();
        Action RedFarMoveToShootingThirdPathEnd = redFarThirdPathEnd.build();
        //RED NEAR
        Action RedNearMoveToShootingFirstPath = redNearFirstPath.build();
        Action RedNearMoveToShootingSecondPath = redNearSecondPath.build();
        Action RedNearMoveToShootingThirdPath = redNearThirdPath.build();
        //BLUE FAR
        Action BlueFarGoToShootingPosition = blueFarMoveToShootingPosition.build();
        Action BlueFarMoveToShootingFirstPath = blueFarFirstPath.build();
        Action BlueFarMoveToShootingSecondPath = blueFarSecondPath.build();
        Action BlueFarMoveToShootingThirdPath = blueFarThirdPath.build();
        Action BlueFarMoveToShootingThirdPathEnd = blueFarThirdPathEnd.build();
        //BLUE NEAR
        Action BlueNearMoveToShootingFirstPath = blueNearFirstPath.build();
        Action BlueNearMoveToShootingSecondPath = blueNearSecondPath.build();
        Action BlueNearMoveToShootingThirdPath = blueNearThirdPath.build();




        waitForStart();

        if (autoSelector == AutoSelector.RED_FAR) {
            Actions.runBlocking(
                    new ParallelAction(
                          r.setTurretStart(),
                          new SequentialAction(//can do turn to first angle here to speed up time

                              r.activateShooter(),
                              RedFarGoToShootingPosition,
                              shoot(),
                                new SleepAction(0.15),
                                new ParallelAction(
                                    RedFarMoveToShootingFirstPath,
                                    intake()
                                ),
                                shoot(),

                                new SleepAction(0.2),
                                new ParallelAction(
                                        RedFarMoveToShootingSecondPath,
                                        intake()
                                ),
                                 shoot(),

                               new SleepAction(0.15),
                                new ParallelAction(
                                        RedFarMoveToShootingThirdPath,
                                        intake()
                              ),
                               shoot(),
                                  new SleepAction(0.15),
                                  RedFarMoveToShootingThirdPathEnd



                        )
                    )
                    );

        }
        //Im pushing htis again
        else if (autoSelector == AutoSelector.RED_NEAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.setTurretStart(),
                            new SequentialAction(
                                    new ParallelAction(
                                            intake(),
                                            RedNearMoveToShootingFirstPath
                                    ),

                            new SleepAction(1),

                            nearShoot(),
                            new SleepAction(1),

                            new ParallelAction(
                                    intake(),
                                    RedNearMoveToShootingSecondPath
                            ),

                            new SleepAction(1),

                            nearShoot(),
                            new SleepAction(1),
                            new ParallelAction(
                                    intake(),
                                    RedNearMoveToShootingThirdPath
                            ),
                            new SleepAction(1),
                            nearShoot()

                    ))
            );

        }
        else if (autoSelector == AutoSelector.BLUE_FAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.setTurretStart(),
                            new SequentialAction(
                                    BlueFarGoToShootingPosition,
                                    shoot(),
                                    new SleepAction(0.15),

                                new ParallelAction(
                                        BlueFarMoveToShootingFirstPath,
                                        intake()
                                ),

                                    shoot(),
                                    new SleepAction(0.15),

                                new ParallelAction(
                                    intake(),
                                    BlueFarMoveToShootingSecondPath
                                ),
                                shoot(),

                                new SleepAction(0.15),

                                new ParallelAction(
                                        intake(),
                                        BlueFarMoveToShootingThirdPath
                                ),
                                shoot(),
                                    new SleepAction(0.15),
                                    BlueFarMoveToShootingThirdPathEnd
                            )
            ));


        }
        else if (autoSelector == AutoSelector.BLUE_NEAR){
            Actions.runBlocking(
                    new ParallelAction(
                            r.setTurretStart(),
                            new SequentialAction(
                                    new SleepAction(1),
                                    new ParallelAction(
                                           intake(),
                                           BlueFarMoveToShootingFirstPath
                                    ),

                            new SleepAction(1),
                            nearShoot(),

                            new SleepAction(1),

                            new ParallelAction(
                                    intake(),
                                    BlueNearMoveToShootingSecondPath
                            ),

                            new SleepAction(1),
                            nearShoot(),
                            new SleepAction(1),

                            new ParallelAction(
                                    intake(),
                                    BlueNearMoveToShootingThirdPath
                            ),
                            new SleepAction(1),
                            nearShoot()
                    )));

        }else
            r.drive.localizer.setPose(blueStartNear);




    }
    public Action shoot(){
        return new SequentialAction(
                r.activateShooter(),
                r.openHoodServo(),
                r.checkShooterVelocity(),
                r.turnOnIntake(),
                r.ballBlockServoOpen()
        );
    }

    public Action nearShoot(){
        return new SequentialAction(
                r.activateShooterNear(),
                r.openHoodServoNear(),
                r.checkShooterVelocityNear(),
                r.turnOnIntake(),
                r.ballBlockServoOpen()

        );
    }
    
    public Action intake(){
        return new SequentialAction(
                r.ballBlockServoBlock(),
                r.closeHoodServo(),
                new SleepAction(0.15),
                r.turnOnIntake()
        );
    }
}

