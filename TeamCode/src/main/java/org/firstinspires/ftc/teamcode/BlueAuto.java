package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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






    @Config
    @Autonomous(name = "BlueAuto")
    public class BlueAuto extends LinearOpMode {


        public enum AutoSelector { BLUE_FAR, BLUE_NEAR}

        public Automonous.AutoSelector autoSelector = Automonous.AutoSelector.BLUE_FAR;
        public HwRobot r = null;
        public static double SHOOTING_DELAY = 0.45;
        public static double SELECTOR_DELAY_TIME = 0.4;

        public static double NEAR_DELAY = 10000;
        public static double MIDDLE_DELAY = 16500;
        public static double FAR_DELAY = 25000;

        public ElapsedTime intakeClock = new ElapsedTime();

        private final ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() throws InterruptedException {
            r = new HwRobot(telemetry, hardwareMap);
            r.init();


            while (opModeInInit()) {

                double time = runtime.seconds();
                telemetry.addData(" AUTO SELECTED", autoSelector);
                telemetry.addLine("D-Pad Up for Blue Far");
                telemetry.addLine("D-Pad Right for Blue Near");
                telemetry.addLine("D-Pad Down for Blue Far 9 Ball");
                telemetry.update();

                if (gamepad1.dpad_up) {

                    autoSelector = Automonous.AutoSelector.BLUE_FAR;

                } else if (gamepad1.dpad_right) {

                    autoSelector = Automonous.AutoSelector.BLUE_NEAR;

                } else if (gamepad1.dpad_down) {

                    autoSelector = Automonous.AutoSelector.BLUE_FAR_9_BALL;

                }

            }

            Pose2d blueStartFar = new Pose2d(63, 12, Math.toRadians(180));

            Pose2d blueStartNear = new Pose2d(-63, 12, Math.toRadians(0));

            Pose2d blueStartFar9Ball = new Pose2d(63, 12, Math.toRadians(180));



            if (autoSelector == Automonous.AutoSelector.BLUE_FAR) {
                r.drive.localizer.setPose(blueStartFar);

            } else if (autoSelector == Automonous.AutoSelector.BLUE_NEAR) {
                r.drive.localizer.setPose(blueStartNear);

            } else if (autoSelector == Automonous.AutoSelector.BLUE_FAR_9_BALL) {
                r.drive.localizer.setPose(blueStartFar);
                
            }


            // BLUE FAR (Change to 6 ball auto later)
            TrajectoryActionBuilder blueFarMoveToShootingPose = r.drive.actionBuilder(blueStartFar)//moveToShootPoseFarBlue
                    .lineToXSplineHeading(-12,Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueFarFirstPath = blueFarMoveToShootingPose.fresh()//firstPathFarBlue
                    .setTangent(Math.toRadians(270))
                    .lineToY(-55)
                    .lineToY(-12)
                    .endTrajectory();



            TrajectoryActionBuilder blueFarSecondPath = blueFarFirstPath.fresh()//secondPathFarBlue
                    .strafeToSplineHeading(new Vector2d(14, -12), Math.toRadians(270))
                    .setTangent(Math.toRadians(90))
                    .strafeToSplineHeading(new Vector2d(14, -58), Math.toRadians(270), new TranslationalVelConstraint(40))
                    .setTangent(Math.toRadians(40))
                    .splineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(270)), Math.toRadians(0))
                    .endTrajectory();





            TrajectoryActionBuilder blueFarThirdPath = blueFarSecondPath.fresh()//thirdPathFarRed
                    .strafeToLinearHeading(new Vector2d(40, 12), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .strafeToLinearHeading(new Vector2d(40,50), Math.toRadians(270), new TranslationalVelConstraint(40))
                    .lineToY(50)
                    .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(270)), Math.toRadians(40))
                    .endTrajectory();


            TrajectoryActionBuilder blueFarThirdPathEnd = blueFarThirdPath.fresh()//endBlueFar
                    .strafeToLinearHeading(new Vector2d(-12, 39), Math.toRadians(270))
                    .endTrajectory();


            //BLUE FAR 9 BALL
            TrajectoryActionBuilder blueFar9BMoveToShootingPose = r.drive.actionBuilder(blueStartFar9Ball)
                    .lineToX(55)

                    .endTrajectory();

            TrajectoryActionBuilder blueFar9BFirstPath = blueFar9BMoveToShootingPose.fresh()
                    .splineToLinearHeading(new Pose2d(35, -35, Math.toRadians(270)), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-55)
                    .splineToLinearHeading(new Pose2d(55, -12, Math.toRadians(210)), Math.toRadians(10))
                    .endTrajectory();

            TrajectoryActionBuilder blueFar9BSecondPath = blueFar9BFirstPath.fresh()
                    .splineToLinearHeading(new Pose2d(12, -35, Math.toRadians(270)), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-55)
                    .strafeToLinearHeading(new Vector2d(55, -12), Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueFar9BThirdPath = blueFar9BSecondPath.fresh()
                    .splineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(270)), Math.toRadians(90))
                    .lineToY(-55)
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(62, -12, Math.toRadians(270)), Math.toRadians(180))

                    .endTrajectory();
            TrajectoryActionBuilder blueFar9BThirdPathEnd = blueFar9BThirdPath.fresh()
                    .strafeToLinearHeading(new Vector2d(35,-12), Math.toRadians(180))
                    .endTrajectory();

            //BLUE NEAR
            TrajectoryActionBuilder blueNearMoveToShootingPose = r.drive.actionBuilder(blueStartNear)//moveToShootPoseNearBlue
                    .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(140))
                    .endTrajectory();



            TrajectoryActionBuilder blueNearFirstPath = blueNearMoveToShootingPose.fresh()//firstPathNearBlue
                    .setTangent(Math.toRadians(90))
                    .lineToY(56)
                    .endTrajectory();

            TrajectoryActionBuilder blueNearSecondPath = blueNearFirstPath.fresh()//secondPathNearBlue
                    .strafeToLinearHeading(new Vector2d(12, 12), Math.toRadians(90))
                    .setTangent(Math.toRadians(90))
                    .lineToY(56)
                    .lineToY(12)
                    .setTangent(Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(140))
                    .endTrajectory();

            TrajectoryActionBuilder blueNearThirdPath = blueNearSecondPath.fresh()//thirdPathNearBlue
                    .strafeToLinearHeading(new Vector2d(36, 12), Math.toRadians(90))
                    .setTangent(Math.toRadians(90))
                    .lineToY(56)
                    .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(140)), Math.toRadians(0))
                    .endTrajectory();

            TrajectoryActionBuilder blueNearThirdPathEnd = blueNearThirdPath.fresh()//endBlueNear


                    .endTrajectory();

            //TrajectoryActionBuilder redNearThirdPathEnd = redNearThirdPath.fresh()//endRedNear
            //.lineToY(38)
            //.endTrajectory();


            //build trajectories
            //Action *NameOfPath* = nameOfPath.build();


            //BLUE FAR
            Action BlueFarGoToShootingPosition = blueFarMoveToShootingPose.build();
            Action BlueFarMoveToShootingFirstPath = blueFarFirstPath.build();
            Action BlueFarMoveToShootingSecondPath = blueFarSecondPath.build();
            Action BlueFarMoveToShootingThirdPath = blueFarThirdPath.build();
            Action BlueFarEnd = blueFarThirdPathEnd.build();

            //BLUE FAR 9 BALL
            Action BlueFar9BallGoToShootingPosition = blueFar9BMoveToShootingPose.build();
            Action BlueFar9BallMoveToShootingFirstPath = blueFar9BFirstPath.build();
            Action BlueFar9BallMoveToShootingSecondPath = blueFar9BSecondPath.build();
            Action BlueFar9BallMoveToShootingThirdPath = blueFar9BThirdPath.build();
            Action BlueFar9BallEnd = blueFar9BThirdPathEnd.build();


            //BLUE NEAR
            Action BlueNearGoToShootingPosition = blueNearMoveToShootingPose.build();
            Action BlueNearMoveToShootingFirstPath = blueNearFirstPath.build();
            Action BlueNearMoveToShootingSecondPath = blueNearSecondPath.build();
            Action BlueNearMoveToShootingThirdPath = blueNearThirdPath.build();
            Action BlueNearEnd= blueNearThirdPathEnd.build();

            waitForStart();

            if (autoSelector == Automonous.AutoSelector.BLUE_FAR) {
                Actions.runBlocking(
                        new ParallelAction(
                                r.turnTurretBlue(),
                                new SequentialAction(//can do turn to first angle here to speed up time

                                        //r.activateShooter(),
                                        BlueFarGoToShootingPosition,
                                        shoot(),
                                        new SleepAction(1.75),
                                        new ParallelAction(
                                                BlueFarMoveToShootingFirstPath,
                                                intake()
                                        ),
                                        shoot(),

                                        new SleepAction(1.75),
                                        new ParallelAction(
                                                BlueFarMoveToShootingSecondPath,
                                                intake()
                                        ),
                                        shoot(),

                                        new SleepAction(1.75),
                                        new ParallelAction(
                                                BlueFarMoveToShootingThirdPath,
                                                intake()
                                        ),
                                        shoot(),
                                        new SleepAction(1.75),
                                        BlueFarEnd


                                )
                        )
                );

            }
            //Im pushing htis again
            else if (autoSelector == Automonous.AutoSelector.BLUE_NEAR) {
                Actions.runBlocking(
                        new ParallelAction(
                                r.setTurretStart(),
                                new SequentialAction(
                                        new ParallelAction(
                                                intake(),
                                                BlueNearMoveToShootingFirstPath
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
                                        nearShoot(),
                                        new ParallelAction(
                                                BlueNearEnd

                                        )

                                ))
                );

            } else
                r.drive.localizer.setPose(blueStartNear);


        }

        public Action shoot() {
            return new SequentialAction(
                    r.turnOnIntake(),
                    r.ballBlockServoOpen()
            );
        }

        public Action nearShoot() {
            return new SequentialAction(
                    r.turnOnIntake(),
                    r.ballBlockServoOpen()

            );
        }

        public Action intake() {
            return new SequentialAction(
                    r.ballBlockServoBlock(),
                    r.turnOnIntake()
            );


        }

    }


