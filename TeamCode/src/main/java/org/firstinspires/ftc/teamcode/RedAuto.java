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


    public enum AutoSelector {RED_FAR, RED_NEAR}
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

            if (gamepad1.dpad_up) {

                autoSelector = Automonous.AutoSelector.RED_FAR;

            } else if (gamepad1.dpad_right) {

                autoSelector = Automonous.AutoSelector.RED_NEAR;

            }

        }

        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-63, 12, Math.toRadians(0));


        if (autoSelector == Automonous.AutoSelector.RED_FAR) {
            r.drive.localizer.setPose(redStartFar);
        } else if (autoSelector == Automonous.AutoSelector.RED_NEAR) {
            r.drive.localizer.setPose(redStartNear);
        }


        // RED FAR (Change to 6 ball auto later)
        TrajectoryActionBuilder redFarMoveToShootingPose = r.drive.actionBuilder(redStartFar)//moveToShootPoseFarRed
                .lineToX(-12)
                //.turnTo(Math.toRadians(140))
                .endTrajectory();

        TrajectoryActionBuilder redFarFirstPath = redFarMoveToShootingPose.fresh()//firstPathFarRed
                .turnTo(Math.toRadians(90))
                .lineToY(55)
                .setTangent(Math.toRadians(90))
                .lineToY(12)
                //.turnTo(Math.toRadians(140))
                .endTrajectory();
        //.setTangent(Math.toRadians(180))
        //.splineToLinearHeading(new Pose2d(36, 30,Math.toRadians(90)),
        //      Math.toRadians(90))
        //.lineToY(55,  new TranslationalVelConstraint(20))
        //.setTangent(Math.toRadians(270))
        //.splineToLinearHeading(new Pose2d(55, 15,Math.toRadians(158)),Math.toRadians(0))
        //.endTrajectory();


        TrajectoryActionBuilder redFarSecondPath = redFarFirstPath.fresh()//secondPathFarRed
                //.turnTo(0)
                //.setTangent(Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(12, 12), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-14, 12, Math.toRadians(140)), Math.toRadians(0))
                .endTrajectory();
        //.setTangent(Math.toRadians(180))
        //.splineToLinearHeading(new Pose2d(13,30, Math.toRadians(90)), Math.toRadians(90))//13,35,90
        //.lineToY(52, new TranslationalVelConstraint(15))
        //.setTangent(Math.toRadians(270))
        //.splineToLinearHeading(new Pose2d(55, 12,Math.toRadians(155)),Math.toRadians(0))



        TrajectoryActionBuilder redFarThirdPath = redFarSecondPath.fresh()//thirdPathFarRed
                .turnTo(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(38, 12), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(57)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12, 12, Math.toRadians(90)), Math.toRadians(0))
                .endTrajectory();
        //.setTangent(Math.toRadians(180))
        //.splineToLinearHeading(new Pose2d(-10, 30,Math.toRadians(90)),Math.toRadians(90))
        //.lineToY(55, new TranslationalVelConstraint(25))
        //.setTangent(Math.toRadians(-45))
        //.splineToLinearHeading(new Pose2d(50, 12,Math.toRadians(155)), Math.toRadians(-30), new TranslationalVelConstraint(55))


        TrajectoryActionBuilder redFarThirdPathEnd = redFarThirdPath.fresh()//endRedFar
                .strafeToLinearHeading(new Vector2d(-12, 39), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFarFourthPath = redFarThirdPathEnd.fresh()



                .endTrajectory();



        //RED NEAR
        TrajectoryActionBuilder redNearMoveToShootingPose = r.drive.actionBuilder(redStartNear)//moveToShootPoseNearRed
                .strafeToLinearHeading(new Vector2d(-12,12),Math.toRadians(90))
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
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redNearThirdPath = redNearSecondPath.fresh()//thirdPathNearRed
                .strafeToSplineHeading(new Vector2d(36, 12), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(56)
                .splineToSplineHeading(new Pose2d(-12, 12, Math.toRadians(90)), Math.toRadians(0))
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
        Action RedFarEnd = redFarThirdPathEnd.build();

        //RED NEAR
        Action RedNearGoToShootingPosition = redNearMoveToShootingPose.build();
        Action RedNearMoveToShootingFirstPath = redNearMoveToShootingPose.build();
        Action RedNearMoveToShootingSecondPath = redNearSecondPath.build();
        Action RedNearMoveToShootingThirdPath = redNearThirdPath.build();
        //Action RedNearEnd = redNearThirdPathEnd.build();

        waitForStart();

        if (autoSelector == Automonous.AutoSelector.RED_FAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(//can do turn to first angle here to speed up time

                                    r.activateShooter(),
                                    RedFarGoToShootingPosition,
                                    shoot(),
                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFarMoveToShootingFirstPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFarMoveToShootingSecondPath,
                                            intake()
                                    ),
                                    shoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedFarMoveToShootingThirdPath,
                                            intake()
                                    ),
                                    shoot(),
                                    new SleepAction(1.75)
                                    //RedFarEnd



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
                                    r.activateShooterNear(),
                                    RedNearGoToShootingPosition,
                                    shoot(),

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
                                            RedNearMoveToShootingSecondPath,
                                            intake()
                                    ),

                                    nearShoot(),

                                    new SleepAction(1.75),
                                    new ParallelAction(
                                            RedNearMoveToShootingThirdPath,
                                            intake()
                                    ),
                                    nearShoot(),
                                    new SleepAction(1.75)

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
            new SleepAction(0.15),
            r.turnOnIntake()
    );








    }

}