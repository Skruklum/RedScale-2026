package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-70, 29, Math.toRadians(0)))
                .splineTo(new Vector2d(-14.1, 4), Math.toRadians(90))
                .waitSeconds(3)
                .lineToY(60)
                .lineToY(4)
                .waitSeconds(3)
                .splineTo(new Vector2d(12.2, 60.1), Math.toRadians(90))
                .strafeTo(new Vector2d(-14.1, 4))
                .waitSeconds(3)
                .splineTo(new Vector2d(37.5, 60.1), Math.toRadians(90))
                .strafeTo(new Vector2d(-14.1, 4))

                .waitSeconds(3)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }
}