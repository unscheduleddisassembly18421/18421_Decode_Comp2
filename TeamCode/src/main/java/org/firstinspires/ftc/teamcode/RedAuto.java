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
@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode{


    public enum AutoSelector {RED_FAR, RED_NEAR, RED_FAR_9_BALL}
    public Automonous.AutoSelector autoSelector = Automonous.AutoSelector.RED_FAR;
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
            telemetry.addLine("D-Pad Up for Red Far");
            telemetry.addLine("D-Pad Right for Red Near");
            telemetry.addLine("D-Pad Down for Red Near 18 Ball");
            telemetry.addLine("D-Pad Left for Red Far 12 Ball");

            telemetry.update();

            if (gamepad1.dpad_up) {

                autoSelector = Automonous.AutoSelector.RED_FAR;

            } else if (gamepad1.dpad_right) {

                autoSelector = Automonous.AutoSelector.RED_NEAR;

            }else if (gamepad1.dpad_down) {

                autoSelector = Automonous.AutoSelector.RED_NEAR_18_BALL;


            }else if (gamepad1.dpad_left) {

                autoSelector =Automonous.AutoSelector.RED_FAR_12_BALL;
            }

        }

        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-50, 52, Math.toRadians(314));

        Pose2d redStartNear18Ball = new Pose2d(63, 12, Math.toRadians(314));

        Pose2d redStartFar12Ball = new Pose2d(-50, 52, Math.toRadians(180));


        if (autoSelector == Automonous.AutoSelector.RED_FAR) {
            r.drive.localizer.setPose(redStartFar);

        } else if (autoSelector == Automonous.AutoSelector.RED_NEAR) {
            r.drive.localizer.setPose(redStartNear);

        } else if (autoSelector == Automonous.AutoSelector.RED_NEAR_18_BALL) {
            r.drive.localizer.setPose(redStartFar);

        } else if (autoSelector == Automonous.AutoSelector.RED_FAR_12_BALL) {
            r.drive.localizer.setPose(redStartFar);
        }


        // RED FAR (COMP 3)
        TrajectoryActionBuilder redFarMoveToShootingPose = r.drive.actionBuilder(redStartFar)//moveToShootPoseFarRed
                .lineToXSplineHeading(57, Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFarFirstPath = redFarMoveToShootingPose.fresh()//firstPathFarRed
                .strafeToSplineHeading(new Vector2d(9, 14), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(9, 53), Math.toRadians(90),
                        new TranslationalVelConstraint(45))
                //.strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))


                //.strafeToSplineHeading(new Vector2d(52, 55), Math.toRadians(70))
                //.strafeToSplineHeading(new Vector2d(62, 55), Math.toRadians(65))
                //.strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redFarSecondPath = redFarFirstPath.fresh()//secondPathFarRed
                .strafeToSplineHeading(new Vector2d(-3, 26),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-3,50),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))

                //.strafeToSplineHeading(new Vector2d(33, 28), Math.toRadians(90))
                //.setTangent(Math.toRadians(90))
                //.lineToY(59)
                //.strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redFarThirdPath = redFarSecondPath.fresh()//thirdPathFarRed
                .strafeToSplineHeading(new Vector2d(33, 28), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(55)
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))


                //.strafeToSplineHeading(new Vector2d(14, 14), Math.toRadians(90))
                //.strafeToSplineHeading(new Vector2d(14, 56), Math.toRadians(90),
                //        new TranslationalVelConstraint(45))

                .endTrajectory();


        TrajectoryActionBuilder redFarFourthPath = redFarThirdPath.fresh()//fourthPathFarRed
                .strafeToSplineHeading(new Vector2d(52, 55), Math.toRadians(70))
                .strafeToSplineHeading(new Vector2d(62, 55), Math.toRadians(65))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))

                //.strafeToSplineHeading(new Vector2d(-3, 26),Math.toRadians(90))
                //.strafeToSplineHeading(new Vector2d(-3,52),Math.toRadians(90))
                //.strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))


                .endTrajectory();

        TrajectoryActionBuilder redFarFifthPath = redFarFourthPath.fresh()//fifthPathFarRed
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(50))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFarSixthPath = redFarFifthPath.fresh()//sixthPathFarRed
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(50))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();
        TrajectoryActionBuilder redFarThirdPathEnd = redFarSixthPath.fresh()


                .endTrajectory();


        //RED NEAR 18 BALL
        TrajectoryActionBuilder redNear18BMoveToShootingPose = r.drive.actionBuilder(redStartNear18Ball)

                .endTrajectory();

        TrajectoryActionBuilder redNear18BFirstPath = redNear18BMoveToShootingPose.fresh()

                .endTrajectory();

        TrajectoryActionBuilder redNear18BSecondPath = redNear18BFirstPath.fresh()

                .endTrajectory();

        TrajectoryActionBuilder redNear18BThirdPath = redNear18BSecondPath.fresh()


                .endTrajectory();

        TrajectoryActionBuilder redNear18BFourthPath = redNear18BThirdPath.fresh()


                .endTrajectory();

        TrajectoryActionBuilder redNear18BFifthPath = redNear18BFourthPath.fresh()

                .endTrajectory();

        TrajectoryActionBuilder redNear18BFifthPathEnd = redNear18BFifthPath.fresh()

                .endTrajectory();


        //RED FAR 12 BALL (COMP 3)
        TrajectoryActionBuilder redFar12BMoveToShootingPose = r.drive.actionBuilder(redStartFar)
                .lineToXSplineHeading(57, Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar12BFirstPath = redFar12BMoveToShootingPose.fresh()
                .strafeToSplineHeading(new Vector2d(34, 28), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(59)
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar12BSecondPath = redFar12BFirstPath.fresh()
                .strafeToSplineHeading(new Vector2d(14, 25), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(59)
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar12BThirdPath = redFar12BSecondPath.fresh()
                .strafeToSplineHeading(new Vector2d(-12, 25), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(59)
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar12BThirdPathEnd = redFar12BThirdPath.fresh()
                .lineToXSplineHeading(38, Math.toRadians(90))
                .endTrajectory();



        //RED NEAR (COMP 3)
        TrajectoryActionBuilder redNearMoveToShootingPose = r.drive.actionBuilder(redStartNear)//moveToShootPoseNearRed
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redNearFirstPath = redNearMoveToShootingPose.fresh()//firstPathNearRed
                .strafeToSplineHeading(new Vector2d(14, 33), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(14, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redNearSecondPath = redNearFirstPath.fresh()//secondPathNearRed
                .strafeToSplineHeading(new Vector2d(33, 25), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(33, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redNearThirdPath = redNearSecondPath.fresh()//thirdPathNearRed
                .strafeToSplineHeading(new Vector2d(4, 12), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(4, 58), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redNearFourthPath = redNearSecondPath.fresh()//thirdPathNearRed
                //.strafeToSplineHeading(new Vector2d(4, 20), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12, 25), Math.toRadians(90),
                        new TranslationalVelConstraint(45))
                .setTangent(Math.toRadians(90))
                .lineToY(58)
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redNearThirdPathEnd = redNearFourthPath.fresh()//endRedNear
                .strafeToSplineHeading(new Vector2d(4, 30), Math.toRadians(90))
                .endTrajectory();


        //build trajectories
        //Action *NameOfPath* = nameOfPath.build();


        //RED FAR
        Action RedFarGoToShootingPosition = redFarMoveToShootingPose.build();
        Action RedFarMoveToShootingFirstPath = redFarFirstPath.build();
        Action RedFarMoveToShootingSecondPath = redFarSecondPath.build();
        Action RedFarMoveToShootingThirdPath = redFarThirdPath.build();
        Action RedFarMoveToShootingFourthPath = redFarFourthPath.build();
        Action RedFarMoveToShootingFifthPath = redFarFifthPath.build();
        Action RedFarMoveToShootingSixthPath = redFarSixthPath.build();
        Action RedFarEnd = redFarThirdPathEnd.build();

        //RED NEAR 18 BALL
        Action RedNear18BallGoToShootingPosition = redNear18BMoveToShootingPose.build();
        Action RedNear18BallMoveToShootingFirstPath = redNear18BFirstPath.build();
        Action RedNear18BallMoveToShootingSecondPath = redNear18BSecondPath.build();
        Action RedNear18BallMoveToShootingThirdPath = redNear18BThirdPath.build();
        Action RedNear18BallMoveToShootingFourthPath = redNear18BFourthPath.build();
        Action RedNear18BallMoveToShootingFifthPath = redNear18BFifthPath.build();
        Action RedNear18BallEnd = redNear18BFifthPathEnd.build();

        //RED FAR 12 BALL (COMP 3)
        Action RedFar12BallGoToShootingPosition = redFar12BMoveToShootingPose.build();
        Action RedFar12BallMoveToShootingFirstPath = redFar12BFirstPath.build();
        Action RedFar12BallMoveToShootingSecondPath = redFar12BSecondPath.build();
        Action RedFar12BallMoveToShootingThirdPath = redFar12BThirdPath.build();
        Action RedFar12BallEnd = redFar12BThirdPathEnd.build();


        //RED NEAR
        Action RedNearGoToShootingPosition = redNearMoveToShootingPose.build();
        Action RedNearMoveToShootingFirstPath = redNearMoveToShootingPose.build();
        Action RedNearMoveToShootingSecondPath = redNearSecondPath.build();
        //Action RedNearMoveToShootingThirdPath = redNearThirdPath.build();
        Action RedNearMoveToShootingFourthPath = redNearFourthPath.build();
        Action RedNearEnd = redNearThirdPathEnd.build();

        waitForStart();

        if (autoSelector == Automonous.AutoSelector.RED_FAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(//can do turn to first angle here to speed up time
                                    new InstantAction(()->r.redGoalAutoPose = new Pose2d(-63, 43,0)),
                                    new ParallelAction(
                                            RedFarGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(0.45),
                                    shoot(),
                                    new SleepAction(0.9),
                                    new InstantAction(()->r.redGoalAutoPose = new Pose2d(-67, 57,0)),
                                    new ParallelAction(
                                            RedFarMoveToShootingFirstPath,
                                            intake()
                                    ),
                                    new ParallelAction(
                                            RedFarMoveToShootingSecondPath,
                                            intake()
                                            ),
                                    shoot(),

                                    new SleepAction(0.9),
                                    new ParallelAction(
                                            RedFarMoveToShootingThirdPath,
                                            intake()
                                    ),
                                    shoot(),
                                    new SleepAction(0.9),
                                    new ParallelAction(
                                            RedFarMoveToShootingFourthPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(0.9),
                                    new ParallelAction(
                                            RedFarMoveToShootingFifthPath,
                                            intake()
                                    ),
                                    shoot(),
                                    new SleepAction(0.9),
                                    new ParallelAction(
                                            RedFarMoveToShootingSixthPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(0.9),
                                    new ParallelAction(
                                            RedFarEnd,
                                            intake()
                                    )


                            )
                    )
            );
        } else if (autoSelector == Automonous.AutoSelector.BLUE_FAR_12_BALL) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(

                                    new ParallelAction(
                                            RedFar12BallGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(0.75),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar12BallMoveToShootingFirstPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar12BallMoveToShootingSecondPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar12BallMoveToShootingThirdPath,
                                            intake()
                                    ),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar12BallEnd
                                    )

                            )
                    )
            );

        } else if (autoSelector == Automonous.AutoSelector.RED_NEAR_18_BALL) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(//can do turn to first angle here to speed up time
                                    new ParallelAction(
                                            RedNear18BallGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(0.75),
                                    shoot(),
                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNear18BallMoveToShootingFirstPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNear18BallMoveToShootingSecondPath,
                                            intake()
                                    ),
                                    shoot(),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNear18BallMoveToShootingThirdPath,
                                            intake()
                                    ),
                                    shoot(),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNear18BallMoveToShootingFourthPath,
                                            intake()
                                    ),
                                    shoot(),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNear18BallMoveToShootingFifthPath,
                                            intake()
                                    ),
                                    shoot(),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNear18BallEnd
                                    )



                            )

                    )
            );
        }
        //Im pushing htis again
        else if (autoSelector == Automonous.AutoSelector.RED_NEAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(
                                    new ParallelAction(
                                            RedNearGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(0.75),
                                    nearShoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNearMoveToShootingFirstPath,
                                            intake()
                                    ),
                                    nearShoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNearMoveToShootingSecondPath,
                                            intake()
                                    ),
                                    nearShoot(),



                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNearMoveToShootingFourthPath,
                                            intake()
                                    ),
                                    nearShoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNearEnd,
                                            intake()
                                    )

                            )
                    )
            );


        }else {
            r.drive.localizer.setPose(redStartNear);
        }

    Pose2DStorage.StordedPose = r.drive.localizer.getPose();


}
public Action shoot(){
    return new SequentialAction(
            r.turnOnIntake(),
            r.ballBlockServoOpen()
    );
}

public Action nearShoot(){
    return new SequentialAction(
            r.turnOnIntake(),
            r.ballBlockServoOpen()

    );
}

public Action intake(){
    return new SequentialAction(
            r.ballBlockServoBlock(),
            r.turnOnIntake()
    );








    }

}