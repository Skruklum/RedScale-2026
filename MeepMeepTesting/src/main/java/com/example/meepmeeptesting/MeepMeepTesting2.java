package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {

    // Helper method to flip Y and Heading
    public static Pose2d mirror(double x, double y, double degrees) {
        return new Pose2d(x, -y, Math.toRadians(-degrees));
    }

    // Helper method for Vectors (position only)
    public static Vector2d mirrorV(double x, double y) {
        return new Vector2d(x, -y);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(200, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Using the mirror functions to flip the original path
        myBot.runAction(myBot.getDrive().actionBuilder(mirror(-70, 29, 90))
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