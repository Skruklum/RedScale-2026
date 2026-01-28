package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red_Scale_Test_Auto", group = "Autonomous")
public class RedScaleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize your MecanumDrive (this contains your 2-dead wheel localizer)
        // Make sure the starting Pose matches your MeepMeep code exactly
        Pose2d initialPose = new Pose2d(-70, 29, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        waitForStart();

        if (isStopRequested()) return;

        // Build and execute the action
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(-13, 29))
                        .strafeTo(new Vector2d(-16, 0))
                        .strafeTo(new Vector2d(13, 29))
                        .strafeTo(new Vector2d(-16, 0))
                        .strafeTo(new Vector2d(37, 29))
                        .strafeTo(new Vector2d(-16, 0))
                        .strafeTo(new Vector2d(39, 30))
                        .build()
        );
    }
}