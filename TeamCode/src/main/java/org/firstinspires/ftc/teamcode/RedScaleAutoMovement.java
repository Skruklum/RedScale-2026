package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red_Scale_Test_Auto_Movement", group = "Autonomous")
public class RedScaleAutoMovement extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize your MecanumDrive (this contains your 2-dead wheel localizer)
        // Make sure the starting Pose matches your MeepMeep code exactly
        Pose2d initialPose = new Pose2d(-70, 29, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        // Build and execute the action
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .stopAndAdd(shooter.setState(true))
                        .strafeTo(new Vector2d(-51, 10))

                        .splineToConstantHeading(new Vector2d(-20, 31.67), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-20, 60.28), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-51, 10))
                            .waitSeconds(0.1)

                        .splineToConstantHeading(new Vector2d(2, 31.67), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(2, 55), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-51, 12))
                            .waitSeconds(0.1)


                        .splineToConstantHeading(new Vector2d(28, 37.67), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(28, 60.28), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-51, 15))
                            .waitSeconds(0.1)

                        .build());


    }
}