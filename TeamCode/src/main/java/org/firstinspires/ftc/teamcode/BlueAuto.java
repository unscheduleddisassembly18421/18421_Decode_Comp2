package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;






    @Config
    @Autonomous(name = "BlueAuto")
    public class BlueAuto extends LinearOpMode {


        public enum AutoSelector {RED_FAR, RED_NEAR, BLUE_FAR, BLUE_NEAR}

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
                telemetry.update();

                if (gamepad1.dpad_up) {

                    autoSelector = Automonous.AutoSelector.BLUE_FAR;

                } else if (gamepad1.dpad_right) {

                    autoSelector = Automonous.AutoSelector.BLUE_NEAR;

                }

            }

            Pose2d blueStartFar = new Pose2d(63, 12, Math.toRadians(180));

            Pose2d blueStartNear = new Pose2d(-63, 12, Math.toRadians(0));


            if (autoSelector == Automonous.AutoSelector.BLUE_FAR) {
                r.drive.localizer.setPose(blueStartFar);
            } else if (autoSelector == Automonous.AutoSelector.BLUE_NEAR) {
                r.drive.localizer.setPose(blueStartNear);
            }


            // BLUE FAR (Change to 6 ball auto later)
            TrajectoryActionBuilder blueFarMoveToShootingPose = r.drive.actionBuilder(blueStartFar)//moveToShootPoseFarRed
                    .lineToX(-12)
                    .turnTo(Math.toRadians(140))
                    .endTrajectory();

            TrajectoryActionBuilder blueFarFirstPath = blueFarMoveToShootingPose.fresh()//firstPathFarRed
                    .turnTo(Math.toRadians(90))
                    .lineToY(55)
                    .lineToY(12)
                    .turnTo(Math.toRadians(140))
                    .endTrajectory();
            //.setTangent(Math.toRadians(180))
            //.splineToLinearHeading(new Pose2d(36, 30,Math.toRadians(90)),
            //      Math.toRadians(90))
            //.lineToY(55,  new TranslationalVelConstraint(20))
            //.setTangent(Math.toRadians(270))
            //.splineToLinearHeading(new Pose2d(55, 15,Math.toRadians(158)),Math.toRadians(0))
            //.endTrajectory();


            TrajectoryActionBuilder blueFarSecondPath = blueFarFirstPath.fresh()//secondPathFarRed
                    .turnTo(Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(12, 12), Math.toRadians(90))
                    .setTangent(Math.toRadians(90))
                    .lineToY(50)
                    .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(140)), Math.toRadians(0))
                    .endTrajectory();
            //.setTangent(Math.toRadians(180))
            //.splineToLinearHeading(new Pose2d(13,30, Math.toRadians(90)), Math.toRadians(90))//13,35,90
            //.lineToY(52, new TranslationalVelConstraint(15))
            //.setTangent(Math.toRadians(270))
            //.splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(155)),Math.toRadians(0))


            TrajectoryActionBuilder blueFarThirdPath = blueFarSecondPath.fresh()//thirdPathFarRed
                    .turnTo(Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(38, 12), Math.toRadians(90))
                    .setTangent(Math.toRadians(90))
                    .lineToY(50)
                    .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(140)), Math.toRadians(0))
                    .endTrajectory();
            //.setTangent(Math.toRadians(180))
            //.splineToLinearHeading(new Pose2d(-10, 30,Math.toRadians(90)),Math.toRadians(90))
            //.lineToY(55, new TranslationalVelConstraint(25))
            //.setTangent(Math.toRadians(-45))
            //.splineToLinearHeading(new Pose2d(50, 12,Math.toRadians(155)), Math.toRadians(-30), new TranslationalVelConstraint(55))


            TrajectoryActionBuilder blueFarThirdPathEnd = blueFarThirdPath.fresh()//endBlueFar
                    .strafeToLinearHeading(new Vector2d(-12, 39), Math.toRadians(90))
                    .endTrajectory();

            TrajectoryActionBuilder blueFarFourthPath = blueFarThirdPathEnd.fresh()


                    .endTrajectory();


            //BLUE NEAR
            TrajectoryActionBuilder blueNearMoveToShootingPose = r.drive.actionBuilder(blueStartNear)//moveToShootPoseNearBlue
                    .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(140))
                    .endTrajectory();
            //.setTangent(Math.toRadians(90))
            //.lineToY(56)
            //.lineToY(18)
            //.turnTo(Math.toRadians(145))


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


