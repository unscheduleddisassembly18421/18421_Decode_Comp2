package org.firstinspires.ftc.teamcode;

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
                telemetry.addLine("D-Pad Down for Blue Near 18 Ball");
                telemetry.addLine("D-Pad Left for Blue Far 12 Ball");
                telemetry.update();

                if (gamepad1.dpad_up) {

                    autoSelector = Automonous.AutoSelector.BLUE_FAR;

                } else if (gamepad1.dpad_right) {

                    autoSelector = Automonous.AutoSelector.BLUE_NEAR;

                } else if (gamepad1.dpad_down) {

                    autoSelector = Automonous.AutoSelector.BLUE_NEAR_18_BALL;

                } else if (gamepad1.dpad_left) {

                    autoSelector = Automonous.AutoSelector.BLUE_FAR_12_BALL;

                }

            }

            Pose2d blueStartFar = new Pose2d(63, -12, Math.toRadians(180));

            Pose2d blueStartNear = new Pose2d(-50, -52, Math.toRadians(50));

            Pose2d blueStartFar9Ball = new Pose2d(63, 12, Math.toRadians(180));

            Pose2d blueStartFar12Ball = new Pose2d(63, -12, Math.toRadians(180));



            if (autoSelector == Automonous.AutoSelector.BLUE_FAR) {
                r.drive.localizer.setPose(blueStartFar);

            } else if (autoSelector == Automonous.AutoSelector.BLUE_NEAR) {
                r.drive.localizer.setPose(blueStartNear);

            } else if (autoSelector == Automonous.AutoSelector.BLUE_NEAR_18_BALL) {
                r.drive.localizer.setPose(blueStartNear);
                
            }else if (autoSelector == Automonous.AutoSelector.BLUE_FAR_12_BALL) {
                r.drive.localizer.setPose(blueStartFar);
            }


            // BLUE FAR (COMP 3)
            TrajectoryActionBuilder blueFarMoveToShootingPose = r.drive.actionBuilder(blueStartFar)//moveToShootPoseFarBlue
                    .lineToXSplineHeading(57, Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueFarFirstPath = blueFarMoveToShootingPose.fresh()//firstPathFarBlue
                    .strafeToSplineHeading(new Vector2d(11, -14), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(11, -56), Math.toRadians(270),
                            new TranslationalVelConstraint(45))
                    //.strafeToSplineHeading(new Vector2d(52, -55), Math.toRadians(270))
                    //.strafeToSplineHeading(new Vector2d(62, -55), Math.toRadians(290))
                    //.strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(295))
                    .endTrajectory();



            TrajectoryActionBuilder blueFarSecondPath = blueFarFirstPath.fresh()//secondPathFarBlue
                    .strafeToSplineHeading(new Vector2d(-5, -26),Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-5,-52),Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))

                    //.strafeToSplineHeading(new Vector2d(33, -28), Math.toRadians(270))
                    //.setTangent(Math.toRadians(270))
                    //.lineToY(-59)
                    //.strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();



            TrajectoryActionBuilder blueFarThirdPath = blueFarSecondPath.fresh()//thirdPathFarRed
                    .strafeToSplineHeading(new Vector2d(35, -28), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-59)
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))

                    //.strafeToSplineHeading(new Vector2d(11, -14), Math.toRadians(270))
                    //.strafeToSplineHeading(new Vector2d(11, -56), Math.toRadians(270),
                    //        new TranslationalVelConstraint(45))
                    .endTrajectory();


            TrajectoryActionBuilder blueFarFourthPath = blueFarThirdPath.fresh()
                    .strafeToSplineHeading(new Vector2d(52, -55), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(62, -55), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))

                    //.strafeToSplineHeading(new Vector2d(-5, -26),Math.toRadians(270))
                    //.strafeToSplineHeading(new Vector2d(-5,-52),Math.toRadians(270))
                    //.strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();


            TrajectoryActionBuilder blueFarFifthPath = blueFarFourthPath.fresh()
                    .strafeToSplineHeading(new Vector2d(56, -55), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();


            TrajectoryActionBuilder blueFarSixthPath = blueFarFifthPath.fresh()
                    .strafeToSplineHeading(new Vector2d(56, -58), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();



            TrajectoryActionBuilder blueFarThirdPathEnd = blueFarSixthPath.fresh()//endBlueFar
                    .strafeToSplineHeading(new Vector2d(38, -12), Math.toRadians(270))
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



            //BLUE NEAR (COMP 3)
            TrajectoryActionBuilder blueNearMoveToShootingPose = r.drive.actionBuilder(blueStartNear)//moveToShootPoseNearBlue
                    .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueNearFirstPath = blueNearMoveToShootingPose.fresh()//firstPathNearBlue
                    .strafeToSplineHeading(new Vector2d(17, -30), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(17, -58), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueNearSecondPath = blueNearFirstPath.fresh()//secondPathNearBlue
                    .strafeToSplineHeading(new Vector2d(40, -23), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(40, -58), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueNearThirdPath = blueNearSecondPath.fresh()//thirdPathNearBlue
                    .strafeToSplineHeading(new Vector2d(4, -12), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(4, -58), Math.toRadians(270))

                    .endTrajectory();

            TrajectoryActionBuilder blueNearFourthPath = blueNearSecondPath.fresh()
                    //.strafeToSplineHeading(new Vector2d(4, -20), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-12, -25), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-58, new TranslationalVelConstraint(40))
                    .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(270))
                    .endTrajectory();



            TrajectoryActionBuilder blueNearThirdPathEnd = blueNearFourthPath.fresh()//endBlueNear
                    .strafeToSplineHeading(new Vector2d(4, -30), Math.toRadians(270))
                    .endTrajectory();



            // BLUE FAR 12 BALL (COMP 3)
            TrajectoryActionBuilder blueFar12BMoveToShootingPose = r.drive.actionBuilder(blueStartFar)
                    .lineToXSplineHeading(57, Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueFar12BFirstPath = blueFar12BMoveToShootingPose.fresh()
                    .strafeToSplineHeading(new Vector2d(34, -28), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-59)
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueFar12BSecondPath = blueFar12BFirstPath.fresh()
                    .strafeToSplineHeading(new Vector2d(14, -25), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-59)
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueFar12BThirdPath = blueFar12BSecondPath.fresh()
                    .strafeToSplineHeading(new Vector2d(-12, -25), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(59)
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))

                    .endTrajectory();

            TrajectoryActionBuilder blueFar12BThirdPathEnd = blueFar12BThirdPath.fresh()
                    .lineToXSplineHeading(38, Math.toRadians(270))
                    .endTrajectory();



            //build trajectories
            //Action *NameOfPath* = nameOfPath.build();


            //BLUE FAR (COMP 3)
            Action BlueFarGoToShootingPosition = blueFarMoveToShootingPose.build();
            Action BlueFarMoveToShootingFirstPath = blueFarFirstPath.build();
            Action BlueFarMoveToShootingSecondPath = blueFarSecondPath.build();
            Action BlueFarMoveToShootingThirdPath = blueFarThirdPath.build();
            Action BlueFarMoveToShootingFourthPath = blueFarFourthPath.build();
            Action BlueFarMoveToShootingFifthPath = blueFarFifthPath.build();
            Action BlueFarMoveToShootingSixthPath = blueFarSixthPath.build();
            Action BlueFarEnd = blueFarThirdPathEnd.build();

            //BLUE FAR 9 BALL
            Action BlueFar9BallGoToShootingPosition = blueFar9BMoveToShootingPose.build();
            Action BlueFar9BallMoveToShootingFirstPath = blueFar9BFirstPath.build();
            Action BlueFar9BallMoveToShootingSecondPath = blueFar9BSecondPath.build();
            Action BlueFar9BallMoveToShootingThirdPath = blueFar9BThirdPath.build();
            Action BlueFar9BallEnd = blueFar9BThirdPathEnd.build();

            //BLUE FAR 12 BALL (COMP 3)
            Action BlueFar12BallGoToShootingPosition = blueFar12BMoveToShootingPose.build();
            Action BlueFar12BallMoveToShootingFirstPath = blueFar12BFirstPath.build();
            Action BlueFar12BallMoveToShootingSecondPath = blueFar12BSecondPath.build();
            Action BlueFar12BallMoveToShootingThirdPath = blueFar12BThirdPath.build();
            Action BlueFar12BallEnd = blueFar12BThirdPathEnd.build();



            //BLUE NEAR (COMP 3)
            Action BlueNearGoToShootingPosition = blueNearMoveToShootingPose.build();
            Action BlueNearMoveToShootingFirstPath = blueNearFirstPath.build();
            Action BlueNearMoveToShootingSecondPath = blueNearSecondPath.build();
            //Action BlueNearMoveToShootingThirdPath = blueNearThirdPath.build();
            Action BlueNearMoveToShootingFourthPath = blueNearFourthPath.build();
            Action BlueNearEnd= blueNearThirdPathEnd.build();

            waitForStart();

            if (autoSelector == Automonous.AutoSelector.BLUE_FAR) {
                Actions.runBlocking(
                        new ParallelAction(
                                r.turnTurretBlue(),
                                new SequentialAction(//can do turn to first angle here to speed up time
                                        new InstantAction(()->r.blueAutoGoalPose = new Pose2d(-70, -67,0)),
                                        //tweak THIS GOAL POSE FOR preloaded FAR SHOOT
                                        new ParallelAction(
                                                BlueFarGoToShootingPosition,
                                                intake()
                                        ),
                                        new SleepAction(0.75),
                                        shoot(),
                                        new SleepAction(2),
                                        new InstantAction(()->r.blueAutoGoalPose = new Pose2d(-70, -67,0)),
                                        //tweak for the rest of the far shoots
                                        new ParallelAction(
                                                BlueFarMoveToShootingFirstPath,
                                                intake()
                                        ),

                                        new SleepAction(0.5),
                                        new ParallelAction(
                                                BlueFarMoveToShootingSecondPath,
                                                intake()
                                        ),
                                        shoot(),

                                        new SleepAction(1),
                                        new ParallelAction(
                                                BlueFarMoveToShootingThirdPath,
                                                intake()
                                        ),
                                        shoot(),


                                        new SleepAction(1),
                                        new ParallelAction(
                                                BlueFarMoveToShootingFourthPath,
                                                intake()
                                        ),
                                        shoot(),

                                        new SleepAction(1),
                                        new ParallelAction(
                                                BlueFarMoveToShootingFifthPath,
                                                intake()
                                        ),
                                        shoot(),

                                        new SleepAction(1.5),
                                        new ParallelAction(
                                                BlueFarMoveToShootingSixthPath,
                                                intake()
                                        ),
                                        shoot(),

                                        new SleepAction(1.5),
                                        new ParallelAction(
                                                BlueFarEnd,
                                                intake()
                                        )


                                )
                        )
                );

            } else if (autoSelector == Automonous.AutoSelector.BLUE_FAR_12_BALL) {
                Actions.runBlocking(
                        new ParallelAction(
                                r.turnTurretBlue(),
                                new SequentialAction(
                                        new ParallelAction(
                                                BlueFar12BallGoToShootingPosition,
                                                intake()
                                        ),
                                        new SleepAction(1),
                                        shoot(),


                                        new SleepAction(1),
                                        new ParallelAction(
                                                BlueFar12BallMoveToShootingFirstPath,
                                                intake()
                                        ),
                                        shoot(),


                                        new SleepAction(1),
                                        new ParallelAction(
                                                BlueFar12BallMoveToShootingSecondPath,
                                                intake()
                                        ),
                                        shoot(),


                                        new SleepAction(1),
                                        new ParallelAction(
                                                BlueFar12BallMoveToShootingThirdPath,
                                                intake()
                                        ),
                                        shoot(),


                                        new SleepAction(1),
                                        new ParallelAction(
                                                BlueFar12BallEnd,
                                                intake()
                                        )


                                )
                        )

                );

            //Im pushing htis again
            }else if (autoSelector == Automonous.AutoSelector.BLUE_NEAR) {
                Actions.runBlocking(
                        //tweak auto goal pose in HwRobot
                        new ParallelAction(
                                r.turnTurretBlue(),
                                new SequentialAction(
                                        new ParallelAction(
                                                BlueNearGoToShootingPosition,
                                                intake()
                                        ),
                                        new SleepAction(0.5),
                                        nearShoot(),

                                        new SleepAction(2),
                                        new ParallelAction(
                                                BlueNearMoveToShootingFirstPath,
                                                intake()
                                        ),
                                        nearShoot(),


                                        new SleepAction(2),
                                        new ParallelAction(
                                                BlueNearMoveToShootingSecondPath,
                                                intake()
                                        ),
                                        nearShoot(),


                                        new SleepAction(2),
                                        new ParallelAction(
                                                BlueNearMoveToShootingFourthPath,
                                                intake()
                                        ),
                                        nearShoot(),
                                        new SleepAction(3.5),
                                        new ParallelAction(
                                                BlueNearEnd,
                                                intake()

                                        )

                                )
                        )
                );

            } else {
                r.drive.localizer.setPose(blueStartNear);
            }

            Pose2DStorage.StordedPose = r.drive.localizer.getPose();


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


