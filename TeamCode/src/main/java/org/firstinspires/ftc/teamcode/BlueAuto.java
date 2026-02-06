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

                } else if (gamepad1.dpad_left) {

                    autoSelector = Automonous.AutoSelector.BLUE_FAR_12_BALL;

                }

            }

            Pose2d blueStartFar = new Pose2d(63, -12, Math.toRadians(180));

            Pose2d blueStartNear = new Pose2d(-63, 12, Math.toRadians(0));

            Pose2d blueStartFar9Ball = new Pose2d(63, 12, Math.toRadians(180));



            if (autoSelector == Automonous.AutoSelector.BLUE_FAR) {
                r.drive.localizer.setPose(blueStartFar);

            } else if (autoSelector == Automonous.AutoSelector.BLUE_NEAR) {
                r.drive.localizer.setPose(blueStartNear);

            } else if (autoSelector == Automonous.AutoSelector.BLUE_FAR_9_BALL) {
                r.drive.localizer.setPose(blueStartFar);
                
            }


            // BLUE FAR (COMP 3)
            TrajectoryActionBuilder blueFarMoveToShootingPose = r.drive.actionBuilder(blueStartFar)//moveToShootPoseFarBlue
                    .lineToXSplineHeading(57, Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueFarFirstPath = blueFarMoveToShootingPose.fresh()//firstPathFarBlue
                    .strafeToSplineHeading(new Vector2d(52, -57), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -57), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();



            TrajectoryActionBuilder blueFarSecondPath = blueFarFirstPath.fresh()//secondPathFarBlue
                    .strafeToSplineHeading(new Vector2d(34, -28), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-59)
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();



            TrajectoryActionBuilder blueFarThirdPath = blueFarSecondPath.fresh()//thirdPathFarRed
                    .strafeToSplineHeading(new Vector2d(-2, -25),Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-2,-53),Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-2, -25), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(14, -25), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-59)
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();


//            TrajectoryActionBuilder blueFarFourthPath = blueFarThirdPath.fresh()
//                    .lineToY(-10)
//                    .strafeToSplineHeading(new Vector2d(24, -30), Math.toRadians(270))
//                    .strafeToSplineHeading(new Vector2d(24, -59), Math.toRadians(270))
//                    .strafeToSplineHeading(new Vector2d(60, -59), Math.toRadians(270))
//                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
//                    .endTrajectory();


            TrajectoryActionBuilder blueFarFifthPath = blueFarThirdPath.fresh()
                    .strafeToSplineHeading(new Vector2d(56, -58), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();


            TrajectoryActionBuilder blueFarSixthPath = blueFarFifthPath.fresh()
                    .strafeToSplineHeading(new Vector2d(56, -58), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();



            TrajectoryActionBuilder blueFarThirdPathEnd = blueFarSixthPath.fresh()//endBlueFar
                    .strafeToSplineHeading(new Vector2d(30, -12), Math.toRadians(270))
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
                    .lineToXSplineHeading(-12, Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueNearFirstPath = blueNearMoveToShootingPose.fresh()//firstPathNearBlue
                    .strafeToSplineHeading(new Vector2d(14, -35), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(14, -58), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(270))

                    .endTrajectory();

            TrajectoryActionBuilder blueNearSecondPath = blueNearFirstPath.fresh()//secondPathNearBlue
                    .strafeToSplineHeading(new Vector2d(35, -12), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(35, -58), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(270))

                    .endTrajectory();

            TrajectoryActionBuilder blueNearThirdPath = blueNearSecondPath.fresh()//thirdPathNearBlue
                    .strafeToSplineHeading(new Vector2d(4, -12), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(4, -58), Math.toRadians(270))

                    .endTrajectory();

            TrajectoryActionBuilder blueNearFourthPath = blueNearThirdPath.fresh()
                    .lineToY(-42)
                    .strafeToSplineHeading(new Vector2d(20, -59), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(60, -59), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(270))

                    .endTrajectory();

            TrajectoryActionBuilder blueNearFifthPath = blueNearFourthPath.fresh()
                    .strafeToSplineHeading(new Vector2d(-12, -58), Math.toRadians(270))
                    .strafeToSplineHeading(new Vector2d(-12, -12), Math.toRadians(270))

                    .endTrajectory();


            TrajectoryActionBuilder blueNearThirdPathEnd = blueNearFifthPath.fresh()//endBlueNear
                    .strafeToSplineHeading(new Vector2d(4, -30), Math.toRadians(270))

                    .endTrajectory();


            // BLUE FAR 12 BALL (COMP 3)
            TrajectoryActionBuilder blueFar12BMoveToShootingPose = r.drive.actionBuilder(blueStartFar)
                    .strafeToSplineHeading(new Vector2d(36, -30), Math.toRadians(270))
                    .setTangent(Math.toRadians(270))
                    .lineToY(-59)
                    .strafeToSplineHeading(new Vector2d(60, -12), Math.toRadians(270))
                    .endTrajectory();

            TrajectoryActionBuilder blueFar12BFirstPath = blueFar12BMoveToShootingPose.fresh()

                    .endTrajectory();

            //build trajectories
            //Action *NameOfPath* = nameOfPath.build();


            //BLUE FAR (COMP 3)
            Action BlueFarGoToShootingPosition = blueFarMoveToShootingPose.build();
            Action BlueFarMoveToShootingFirstPath = blueFarFirstPath.build();
            Action BlueFarMoveToShootingSecondPath = blueFarSecondPath.build();
            Action BlueFarMoveToShootingThirdPath = blueFarThirdPath.build();
//            Action BlueFarMoveToShootingFourthPath = blueFarFourthPath.build();
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


            //BLUE NEAR (COMP 3)
            Action BlueNearGoToShootingPosition = blueNearMoveToShootingPose.build();
            Action BlueNearMoveToShootingFirstPath = blueNearFirstPath.build();
            Action BlueNearMoveToShootingSecondPath = blueNearSecondPath.build();
            Action BlueNearMoveToShootingThirdPath = blueNearThirdPath.build();
            Action BlueNearMoveToShootingFourthPath = blueNearFourthPath.build();
            Action BlueNearMoveToShootingFifthPath = blueNearFifthPath.build();
            Action BlueNearEnd= blueNearThirdPathEnd.build();

            waitForStart();

            if (autoSelector == Automonous.AutoSelector.BLUE_FAR) {
                Actions.runBlocking(
                        new ParallelAction(
                                r.turnTurretBlue(),
                                new SequentialAction(//can do turn to first angle here to speed up time
                                        new ParallelAction(
                                                BlueFarGoToShootingPosition,
                                                intake()
                                        ),
                                        new SleepAction(0.1),
                                        shoot(),
                                        new SleepAction(2),
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
                                        new ParallelAction(
                                                BlueFarMoveToShootingFifthPath,
                                                intake()
                                        ),
                                        shoot(),

                                        new SleepAction(1.75),
                                        new ParallelAction(
                                                BlueFarMoveToShootingSixthPath,
                                                intake()
                                        ),
                                        shoot(),

                                        new SleepAction(1.75),
                                        new ParallelAction(
                                                BlueFarEnd,
                                                intake()
                                        )


                                )
                        )
                );

            }
            //Im pushing htis again
            else if (autoSelector == Automonous.AutoSelector.BLUE_NEAR) {
                Actions.runBlocking(
                        new ParallelAction(
                                //r.turnTurretBlue(),
                                new SequentialAction(

                                        new ParallelAction(
                                                //intake(),
                                                BlueNearGoToShootingPosition

                                        ),
                                        //nearShoot(),
                                        new SleepAction(1.75),

                                        new ParallelAction(
                                                //intake(),
                                                BlueNearMoveToShootingSecondPath
                                        ),
                                        //Shoot(),
                                        new SleepAction(1.75),
                                        new ParallelAction(
                                                //intake(),
                                                BlueNearMoveToShootingThirdPath
                                        ),
                                        //Shoot(),
                                        new SleepAction(1.75),
                                        new ParallelAction(
                                                BlueNearEnd
                                                //intake()

                                        )

                                ))
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


