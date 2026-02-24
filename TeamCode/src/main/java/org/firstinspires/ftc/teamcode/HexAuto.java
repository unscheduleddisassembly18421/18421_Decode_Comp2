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
@Autonomous(name = "HexAuto")
public class HexAuto extends LinearOpMode{


    public enum AutoSelector {RED_FAR, RED_NEAR, RED_FAR_12_BALL, RED_FAR_18_BALL,}
    public HexAuto.AutoSelector autoSelector = AutoSelector.RED_FAR;
    public HwRobot r = null;


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

                autoSelector = HexAuto.AutoSelector.RED_FAR;

            }

        }

        Pose2d redStartFar = new Pose2d(63, 16, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-50, 52, Math.toRadians(314));

        Pose2d redStartFar18Ball = new Pose2d(63, 16, Math.toRadians(314));

        Pose2d redStartFar12Ball = new Pose2d(-50, 52, Math.toRadians(180));


        if (autoSelector == HexAuto.AutoSelector.RED_FAR) {
            r.drive.localizer.setPose(redStartFar);

        }

        // HEX RED FAR (STATES)
        TrajectoryActionBuilder redFarMoveToShootingPose = r.drive.actionBuilder(redStartFar)//moveToShootPoseFarRed
                .lineToXSplineHeading(57, Math.toRadians(90))
                .endTrajectory();

        TrajectoryActionBuilder redFarFirstPath = redFarMoveToShootingPose.fresh()//firstPathFarRed
                .strafeToSplineHeading(new Vector2d(55, 55), Math.toRadians(60))
                .strafeToSplineHeading(new Vector2d(64, 60), Math.toRadians(60))
                .strafeToSplineHeading(new Vector2d(60, 12), Math.toRadians(90))
                .endTrajectory();



        TrajectoryActionBuilder redFarThirdPathEnd = redFarFirstPath.fresh()
                .strafeToSplineHeading(new Vector2d(60, 55), Math.toRadians(90))

                .endTrajectory();




        //build trajectories
        //Action *NameOfPath* = nameOfPath.build();


        //RED FAR
        Action RedFarGoToShootingPosition = redFarMoveToShootingPose.build();
        Action RedFarMoveToShootingFirstPath = redFarFirstPath.build();
        Action RedFarEnd = redFarThirdPathEnd.build();


        waitForStart();

        if (autoSelector == HexAuto.AutoSelector.RED_FAR) {
            Actions.runBlocking(
                    new ParallelAction(
                            r.turnTurretRed(),
                            new SequentialAction(//can do turn to first angle here to speed up time
                                    new InstantAction(()->r.redGoalAutoPose = new Pose2d(-74, 65,0)),
                                    //tweak this goal pose for preloaded far shoot
                                    new ParallelAction(
                                            RedFarGoToShootingPosition,
                                            intake()
                                    ),
                                    new SleepAction(1),
                                    shoot(),

                                    new SleepAction(0.95),
                                    new InstantAction(()->r.redGoalAutoPose = new Pose2d(-74, 70,0)),
                                    //tweak this goal pose for rest of far shoots
                                    new ParallelAction(
                                            RedFarMoveToShootingFirstPath,
                                            intake()
                                    ),


                                    new SleepAction(0.925),
                                    new ParallelAction(
                                            RedFarEnd,
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