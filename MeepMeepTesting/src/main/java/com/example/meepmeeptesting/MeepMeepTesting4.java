package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting4 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(300, 250, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 35, Math.toRadians(90)))
                        .strafeTo(new Vector2d(-30, 30))
                        .splineToConstantHeading(new Vector2d(-11.27, 25), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(-11.21, 55), Math.toRadians(90.00))
                        .strafeTo(new Vector2d(-30, 30))
                        .splineToConstantHeading(new Vector2d(15, 25), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(15, 50), Math.toRadians(90.00))
                        .strafeTo(new Vector2d(-30, 30))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }
}