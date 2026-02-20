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


    public enum AutoSelector {RED_FAR, RED_NEAR, RED_FAR_12_BALL, RED_FAR_18_BALL,}
    public RedAuto.AutoSelector autoSelector = AutoSelector.RED_FAR;
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

                autoSelector = RedAuto.AutoSelector.RED_FAR;

            } else if (gamepad1.dpad_right) {

                autoSelector = RedAuto.AutoSelector.RED_NEAR;

            }else if (gamepad1.dpad_down) {

                autoSelector = RedAuto.AutoSelector.RED_FAR_18_BALL;


            }else if (gamepad1.dpad_left) {

                autoSelector = RedAuto.AutoSelector.RED_FAR_12_BALL;

                //}else if (gamepad1.xWasPressed())
            }

        }

        Pose2d redStartFar = new Pose2d(63, 14, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-50, 52, Math.toRadians(314));

        Pose2d redStartFar18Ball = new Pose2d(63, 14, Math.toRadians(314));

        Pose2d redStartFar12Ball = new Pose2d(-50, 52, Math.toRadians(180));


        if (autoSelector == RedAuto.AutoSelector.RED_FAR) {
            r.drive.localizer.setPose(redStartFar);

        } else if (autoSelector == RedAuto.AutoSelector.RED_NEAR) {
            r.drive.localizer.setPose(redStartNear);

        } else if (autoSelector == RedAuto.AutoSelector.RED_FAR_18_BALL) {
            r.drive.localizer.setPose(redStartFar);

        } else if (autoSelector == RedAuto.AutoSelector.RED_FAR_12_BALL) {
            r.drive.localizer.setPose(redStartFar);
        }


        // RED FAR (STATES)
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
                //.strafeToSplineHeading(new Vector2d(62, 55), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(62, 55), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))

                //.strafeToSplineHeading(new Vector2d(-3, 26),Math.toRadians(90))
                //.strafeToSplineHeading(new Vector2d(-3,52),Math.toRadians(90))
                //.strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))


                .endTrajectory();

        TrajectoryActionBuilder redFarFifthPath = redFarFourthPath.fresh()//fifthPathFarRed
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFarSixthPath = redFarFifthPath.fresh()//sixthPathFarRed
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();
        TrajectoryActionBuilder redFarThirdPathEnd = redFarSixthPath.fresh()


                .endTrajectory();


        //RED FAR 18 BALL (STATES)
        TrajectoryActionBuilder redFar18BMoveToShootingPose = r.drive.actionBuilder(redStartFar18Ball) // shoots near preloaded(move to shoot pose) red far 18B
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar18BFirstPath = redFar18BMoveToShootingPose.fresh() // intakes 3rd stack & shoots near (first path) red far 18B
                .setTangent(Math.toRadians(90))
                .lineToY(59) // .strafeToSplineHeading(new Vector2d(-12, 59), Math.toRadians(90))
                .lineToY(12) //  .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar18BSecondPath = redFar18BFirstPath.fresh() // intakes 2nd stack (second path) red far 18B
                .strafeToSplineHeading(new Vector2d(9, 25), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(9, 53), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar18BThirdPath = redFar18BSecondPath.fresh() // opens the gate and shoots far 2nd stack (third path) red far 18B
                .strafeToSplineHeading(new Vector2d(-4,57),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-4, 52), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))

                .endTrajectory();

        TrajectoryActionBuilder redFar18BFourthPath = redFar18BThirdPath.fresh() // intakes 1st stack & shoots far (fourth path) red far 18B
                .strafeToSplineHeading(new Vector2d(33, 28), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(55)
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar18BFifthPath = redFar18BFourthPath.fresh() // intakes & shoots far human player zone (fifth path) red far 18B
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar18BFifthPathEnd = redFar18BFifthPath.fresh() // intakes & shoots far human player zone overflow (fifth path end) red far 18B
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();




        //RED FAR 12 BALL (COMP 3)
        TrajectoryActionBuilder redFar12BMoveToShootingPose = r.drive.actionBuilder(redStartFar) // shoots far preloaded (move to shoot pose) red far 12B
                .lineToXSplineHeading(57, Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar12BFirstPath = redFar12BMoveToShootingPose.fresh() // intakes & shoots far human player zone (first path) red far 12B
                .strafeToSplineHeading(new Vector2d(62, 55), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))

                .endTrajectory();

        TrajectoryActionBuilder redFar12BSecondPath = redFar12BFirstPath.fresh() // intakes 1st stack & shoots far (second path) red far 12B
                .strafeToSplineHeading(new Vector2d(33, 28), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(55)
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redFar12BThirdPath = redFar12BSecondPath.fresh() // intakes 2nd stack & shoots far (third path) red far 12B
                .strafeToSplineHeading(new Vector2d(9, 14), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(9, 53), Math.toRadians(90),
                        new TranslationalVelConstraint(45))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redFar12BFourthPath = redFar12BThirdPath.fresh() // intakes 3rd stack & shoots far (fourth path) red far 12B
                .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12, 30), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12, 54), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redFar12BFourthPathEnd = redFar12BFourthPath.fresh() // leaves far shoot zone (fourth path end) red far 12B
                .lineToXSplineHeading(35, Math.toRadians(90))
                .endTrajectory();



        //RED NEAR (STATES)
        TrajectoryActionBuilder redNearMoveToShootingPose = r.drive.actionBuilder(redStartNear)//moveToShootPoseNearRed
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redNearFirstPath = redNearMoveToShootingPose.fresh()//firstPathNearRed
                .strafeToSplineHeading(new Vector2d(20, 30), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(20, 70), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redNearSecondPath = redNearFirstPath.fresh()//secondPathNearRed
                .setTangent(Math.toRadians(90))
                .lineToY(65, new TranslationalVelConstraint(30))
                .strafeToSplineHeading(new Vector2d(-12, 40), Math.toRadians(90))

                .endTrajectory();

        TrajectoryActionBuilder redNearThirdPath = redNearSecondPath.fresh()//thirdPathNearRed  * Opens the gate, don't use

                .strafeToSplineHeading(new Vector2d(10, 60), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redNearFourthPath = redNearThirdPath.fresh()//thirdPathNearRed
                 .strafeToSplineHeading(new Vector2d(42, 23), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(42, 70), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redNearFifthPath =redNearFourthPath.fresh()
                .strafeToSplineHeading(new Vector2d(14, 55), Math.toRadians(40))
                .strafeToSplineHeading(new Vector2d(55, 55), Math.toRadians(40))
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redNearThirdPathEnd = redNearFourthPath.fresh()//endRedNear
                .strafeToSplineHeading(new Vector2d(10, 30), Math.toRadians(90))
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

        //RED FAR 18 BALL
        Action RedFar18BallGoToShootingPosition = redFar18BMoveToShootingPose.build();
        Action RedFar18BallMoveToShootingFirstPath = redFar18BFirstPath.build();
        Action RedFar18BallMoveToShootingSecondPath = redFar18BSecondPath.build();
        Action RedFar18BallMoveToShootingThirdPath = redFar18BThirdPath.build();
        Action RedFar18BallMoveToShootingFourthPath = redFar18BFourthPath.build();
        Action RedFar18BallMoveToShootingFifthPath = redFar18BFifthPath.build();
        Action RedFar18BallEnd = redFar18BFifthPathEnd.build();

        //RED FAR 12 BALL (COMP 3)
        Action RedFar12BallGoToShootingPosition = redFar12BMoveToShootingPose.build();
        Action RedFar12BallMoveToShootingFirstPath = redFar12BFirstPath.build();
        Action RedFar12BallMoveToShootingSecondPath = redFar12BSecondPath.build();
        Action RedFar12BallMoveToShootingThirdPath = redFar12BThirdPath.build();
        Action RedFar12BallMoveToShootingFourthPath = redFar12BFourthPath.build();
        Action RedFar12BallEnd = redFar12BFourthPathEnd.build();


        //RED NEAR
        Action RedNearGoToShootingPosition = redNearMoveToShootingPose.build();
        Action RedNearMoveToShootingFirstPath = redNearFirstPath.build();
        Action RedNearMoveToShootingSecondPath = redNearSecondPath.build();
        Action RedNearMoveToShootingThirdPath = redNearThirdPath.build();
        Action RedNearMoveToShootingFourthPath = redNearFourthPath.build();
        Action RedNearMoveToShootingFifthPath = redNearFifthPath.build();
        Action RedNearEnd = redNearThirdPathEnd.build();

        waitForStart();

        if (autoSelector == RedAuto.AutoSelector.RED_FAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(//can do turn to first angle here to speed up time
                                    new InstantAction(()->r.redGoalAutoPose = new Pose2d(-74, 60,0)),
                                    //tweak this goal pose for preloaded far shoot
                                    new ParallelAction(
                                            RedFarGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(0.5),
                                    shoot(),

                                    new SleepAction(0.9),
                                    new InstantAction(()->r.redGoalAutoPose = new Pose2d(-74, 70,0)),
                                    //tweak this goal pose for rest of far shoots
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
        } else if (autoSelector == RedAuto.AutoSelector.RED_FAR_12_BALL) {
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

        } else if (autoSelector == RedAuto.AutoSelector.RED_FAR_18_BALL) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(//can do turn to first angle here to speed up time
                                    new InstantAction(()->r.redGoalAutoPose = new Pose2d(-74, 49,0)),
                                    //tweak this goal pose for preloaded far shoot

                                    new ParallelAction(
                                            RedFar18BallGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(0.75),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new InstantAction(()->r.redGoalAutoPose = new Pose2d(-74, 57,0)),
                                    //tweak this goal pose for rest of far shoots

                                    new ParallelAction(
                                            RedFar18BallMoveToShootingFirstPath,
                                            intake()
                                    ),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar18BallMoveToShootingSecondPath,
                                            intake()
                                    ),
                                    shoot(),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar18BallMoveToShootingThirdPath,
                                            intake()
                                    ),
                                    shoot(),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar18BallMoveToShootingFourthPath,
                                            intake()
                                    ),
                                    shoot(),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar18BallMoveToShootingFifthPath,
                                            intake()
                                    ),
                                    shoot(),


                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar18BallEnd
                                    )



                            )

                    )
            );
        }
        //Im pushing htis again
        else if (autoSelector == RedAuto.AutoSelector.RED_NEAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(
                                    new ParallelAction(
                                            RedNearGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(0.25),
                                    nearShoot(),

                                    new SleepAction(1),
                                    new ParallelAction(
                                            RedNearMoveToShootingFirstPath,
                                            intake()
                                    ),
                                    nearShoot(),

                                    new SleepAction(1),
                                    new ParallelAction(
                                            RedNearMoveToShootingSecondPath,
                                            intake()
                                    ),

                                    new SleepAction(1),
                                    new ParallelAction(
                                            RedNearMoveToShootingThirdPath,
                                            intake()
                                    ),
                                    nearShoot(),


                                    new SleepAction(1.1),
                                    new ParallelAction(
                                            RedNearMoveToShootingFourthPath,
                                            intake()
                                    ),
                                    nearShoot(),



                                    new SleepAction(1.1),
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