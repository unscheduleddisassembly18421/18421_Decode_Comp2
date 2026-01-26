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
@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode{


    public enum AutoSelector {RED_FAR, RED_NEAR, RED_FAR_9_BALL}
    public Automonous.AutoSelector autoSelector = Automonous.AutoSelector.RED_FAR_9_BALL;
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
            telemetry.addLine("D-Pad Down for Red Far 9 Ball");
            telemetry.update();

            if (gamepad1.dpad_up) {

                autoSelector = Automonous.AutoSelector.RED_FAR;

            } else if (gamepad1.dpad_right) {

                autoSelector = Automonous.AutoSelector.RED_NEAR;

            }else if  (gamepad1.dpad_down) {

                autoSelector = Automonous.AutoSelector.RED_FAR_9_BALL;


            }

        }

        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-63, 12, Math.toRadians(0));

        Pose2d redStartFar9Ball = new Pose2d(63, 12, Math.toRadians(180));


        if (autoSelector == Automonous.AutoSelector.RED_FAR) {
            r.drive.localizer.setPose(redStartFar);

        } else if (autoSelector == Automonous.AutoSelector.RED_NEAR) {
            r.drive.localizer.setPose(redStartNear);

        } else if (autoSelector == Automonous.AutoSelector.RED_FAR_9_BALL) {
            r.drive.localizer.setPose(redStartFar);
        }


        // RED FAR
        TrajectoryActionBuilder redFarMoveToShootingPose = r.drive.actionBuilder(redStartFar)//moveToShootPoseFarRed
                .lineToXSplineHeading(60, Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFarFirstPath = redFarMoveToShootingPose.fresh()//firstPathFarRed

                .setTangent(Math.toRadians(90))
                .lineToYSplineHeading(56, Math.toRadians(90))
                .lineToYSplineHeading(12, Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redFarSecondPath = redFarFirstPath.fresh()//secondPathFarRed
                .strafeToSplineHeading(new Vector2d(36, 30), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(59)
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();



        TrajectoryActionBuilder redFarThirdPath = redFarSecondPath.fresh()//thirdPathFarRed
                .strafeToSplineHeading(new Vector2d(0, 20),Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(0,58),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,20), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();


        TrajectoryActionBuilder redFarFourthPath = redFarThirdPath.fresh()//fourthPathFarRed
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFarFifthPath = redFarFourthPath.fresh()//fifthPathFarRed
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFarSixthPath = redFarFifthPath.fresh()
                .strafeToSplineHeading(new Vector2d(56, 58), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();


        //RED FAR 9 BALL
        TrajectoryActionBuilder redFar9BMoveToShootingPose = r.drive.actionBuilder(redStartFar9Ball)
                .lineToXSplineHeading(59,Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar9BFirstPath = redFar9BMoveToShootingPose.fresh()
                .splineToLinearHeading(new Pose2d(38, 12, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(55)
                .splineToLinearHeading(new Pose2d(62,12, Math.toRadians(90)), Math.toRadians(10))
                .endTrajectory();

        TrajectoryActionBuilder redFar9BSecondPath = redFar9BFirstPath.fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(12, 12, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(55)
                .strafeToLinearHeading(new Vector2d(62, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFar9BThirdPath = redFar9BSecondPath.fresh()
                .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(62, 12, Math.toRadians(90)), Math.toRadians(-30))

                .endTrajectory();

        TrajectoryActionBuilder redFar9BThirdPathEnd = redFar9BThirdPath.fresh()
                .strafeToLinearHeading(new Vector2d(35, 12), Math.toRadians(90))

                .endTrajectory();





        //RED NEAR
        TrajectoryActionBuilder redNearMoveToShootingPose = r.drive.actionBuilder(redStartNear)//moveToShootPoseNearRed
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(140))
                .endTrajectory();
        //.setTangent(Math.toRadians(90))
        //.lineToY(56)
        //.lineToY(18)
        //.turnTo(Math.toRadians(145))


        TrajectoryActionBuilder redNearFirstPath = redNearMoveToShootingPose.fresh()//firstPathNearRed
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .endTrajectory();

        TrajectoryActionBuilder redNearSecondPath = redNearFirstPath.fresh()//secondPathNearRed
                .strafeToLinearHeading(new Vector2d(12, 12), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .lineToY(12)
                .setTangent(Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(140))
                .endTrajectory();

        TrajectoryActionBuilder redNearThirdPath = redNearSecondPath.fresh()//thirdPathNearRed
                .strafeToLinearHeading(new Vector2d(36, 12), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(140)), Math.toRadians(0))
                .endTrajectory();

        TrajectoryActionBuilder redNearThirdPathEnd = redNearThirdPath.fresh()//endRedNear
                .setTangent(Math.toRadians(90))
                .lineToY(38)
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

        //RED FAR 9 BALL
        Action RedFar9BallGoToShootingPosition = redFar9BMoveToShootingPose.build();
        Action RedFar9BallMoveToShootingFirstPath = redFar9BFirstPath.build();
        Action RedFar9BallMoveToShootingSecondPath = redFar9BSecondPath.build();
        Action RedFar9BallMoveToShootingThirdPath = redFar9BThirdPath.build();
        Action RedFar9BallEnd = redFar9BThirdPathEnd.build();


        //RED NEAR
        Action RedNearGoToShootingPosition = redNearMoveToShootingPose.build();
        Action RedNearMoveToShootingFirstPath = redNearMoveToShootingPose.build();
        Action RedNearMoveToShootingSecondPath = redNearSecondPath.build();
        Action RedNearMoveToShootingThirdPath = redNearThirdPath.build();
        Action RedNearEnd = redNearThirdPathEnd.build();

        waitForStart();

        if (autoSelector == Automonous.AutoSelector.RED_FAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(//can do turn to first angle here to speed up time
                                    new ParallelAction(
                                            RedFarGoToShootingPosition
                                            //intake()
                                    ),
                                    shoot(),
                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFarMoveToShootingFirstPath
                                            //intake()
                                    ),
                                    //shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFarMoveToShootingSecondPath
                                            //intake()
                                    ),
                                    //shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFarMoveToShootingThirdPath
                                            //intake()
                                    ),
                                    //shoot(),
                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFarMoveToShootingFourthPath
                                            //intake()
                                    ),
                                    new ParallelAction(
                                            RedFarMoveToShootingFifthPath
                                    ),
                                    new ParallelAction(
                                            RedFarMoveToShootingSixthPath
                                    )



                            )
                    )
            );

        } else if (autoSelector == Automonous.AutoSelector.RED_FAR_9_BALL) {
            Actions.runBlocking(new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(//can do turn to first angle here to speed up time
                                    new ParallelAction(
                                            RedFar9BallGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(0.75),
                                    shoot(),
                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar9BallMoveToShootingFirstPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar9BallMoveToShootingSecondPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFar9BallMoveToShootingThirdPath,
                                            intake()
                                    ),
                                    shoot(),
                                    new SleepAction(1.75),
                                    RedFar9BallEnd



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

        }else
            r.drive.localizer.setPose(redStartNear);




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