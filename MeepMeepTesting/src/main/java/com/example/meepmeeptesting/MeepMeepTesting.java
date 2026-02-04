package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-70, 29, Math.toRadians(90)))
                .strafeTo(new Vector2d(-51, 10))

                .splineToConstantHeading(new Vector2d(-20, 31.67), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-20, 55.28), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-51, 10))
                    .waitSeconds(0.1)

                .splineToConstantHeading(new Vector2d(2, 31.67), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(2, 55.28), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-51, 10))
                    .waitSeconds(0.1)

                .splineToConstantHeading(new Vector2d(28, 31.67), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(28, 55.28), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-51, 10))
                    .waitSeconds(0.1)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }
}