package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Movement", group = "Autonomous")
public class RedScaleAutoMovement extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize your MecanumDrive (this contains your 2-dead wheel localizer)
        // Make sure the starting Pose matches your MeepMeep code exactly
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        waitForStart();

        if (isStopRequested()) return;

        // Build and execute the action
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(30, 0))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(30, 30))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(30, 0))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(0, 0))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(30, 0))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(30, -30))


                        .build());


    }
}