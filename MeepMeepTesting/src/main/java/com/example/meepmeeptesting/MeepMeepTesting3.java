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


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-70, 29, Math.toRadians(90)))
                .strafeTo(new Vector2d(-51, 4))
                .waitSeconds(3)
                .strafeTo(new Vector2d(-20, 30))
                .strafeTo(new Vector2d(-20, 65))
                //  .splineTo(new Vector2d(12, 60), Math.toRadians(90))
                .strafeTo(new Vector2d(-51, 4))
                .waitSeconds(3)
                //.splineTo(new Vector2d(37.5, 60), Math.toRadians(90))
                .strafeTo(new Vector2d(30, 30))
                .strafeTo(new Vector2d(30, 65))
                .strafeTo(new Vector2d(-51, 4))


                .waitSeconds(3)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }
}