package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Movement Houston", group = "Autonomous")
public class RedScaleAutoMovementHouston extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize your MecanumDrive (this contains your 2-dead wheel localizer)
        // Make sure the starting Pose matches your MeepMeep code exactly
        Pose2d initialPose = (new Pose2d(-48, 48, Math.toRadians(135)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        if (isStopRequested()) return;

        // Build and execute the action
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
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


    }
}