package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-61.85, 32.01, Math.toRadians(90)))
                .strafeTo(new Vector2d(-11, 10))


                .splineToConstantHeading(new Vector2d(-12.95, 30.56), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-12.95, 55.63), Math.toRadians(90.00))
                .strafeTo(new Vector2d(-11, 10))


                .splineToConstantHeading(new Vector2d(12.95, 29.94), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12.54, 55.42), Math.toRadians(90))
                .strafeTo(new Vector2d(-11, 10))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }
}