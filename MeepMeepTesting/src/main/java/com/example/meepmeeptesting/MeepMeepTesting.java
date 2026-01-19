package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        Pose2d redStartFar = new Pose2d(63, 12, Math.toRadians(180));

        Pose2d redStartNear = new Pose2d(-63, 12, Math.toRadians(0));

        Pose2d blueStartFar = new Pose2d(63, -12, Math.toRadians(180));

        Pose2d blueStartNear = new Pose2d(-63, -12, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        String determinesAuto = "redStartFar";

        if (determinesAuto.equals("redStartFar")){
            myBot.runAction(myBot.getDrive().actionBuilder(redStartFar)
                    //.splineToLinearHeading()

                    .splineToLinearHeading(new Pose2d(36, 30, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(55, 15, Math.toRadians(158)), Math.toRadians(0))

                    //.lineToX(30)
                    //.turn(Math.toRadians(90))
                    //.lineToY(30)
                    //.turn(Math.toRadians(90))
                    //.lineToX(0)
                    //.turn(Math.toRadians(90))
                    //.lineToY(0)
                    //.turn(Math.toRadians(90))
                    .build());


            meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();

        }else if (determinesAuto.equals("redStartNear"))



        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
