package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, 48, Math.toRadians(135)))
                .strafeTo(new Vector2d(-12, 12))
                    .waitSeconds(0.5)

                .strafeToLinearHeading(new Vector2d(-11, 26), Math.toRadians(90), new TranslationalVelConstraint(45))
                .strafeToLinearHeading(new Vector2d(-11, 56), Math.toRadians(90), new TranslationalVelConstraint(75))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(135), new TranslationalVelConstraint(125))
                    .waitSeconds(0.5)

                .splineToLinearHeading(new Pose2d(13.18, 22.62, Math.toRadians(90.00)), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(12, 56), Math.toRadians(90), new TranslationalVelConstraint(75))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(135), new TranslationalVelConstraint(125))

                    .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(36, 27), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(36, 57), Math.toRadians(90), new TranslationalVelConstraint(75))
                .strafeToLinearHeading(new Vector2d(-12, 12), Math.toRadians(135), new TranslationalVelConstraint(125))




                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .addEntity(myBot)
                .start();
    }
}