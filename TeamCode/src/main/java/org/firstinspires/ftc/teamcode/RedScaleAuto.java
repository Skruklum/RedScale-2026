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
        Pose2d initialPose = new Pose2d(-70, 29, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        // Build and execute the action
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .afterTime(0, intake.setPower(1.0))
                        .afterTime(0, shooter.setPower(1.0))
                        .splineTo(new Vector2d(-11, 4), Math.toRadians(90))
                        .waitSeconds(3)
                        .lineToY(60)
                        .lineToY(4)
                        .waitSeconds(3)
                        .splineTo(new Vector2d(15, 60), Math.toRadians(90))
                        .strafeTo(new Vector2d(-11, 4))
                        .waitSeconds(3)
                        //.splineTo(new Vector2d(37.5, 60), Math.toRadians(90))
                        .strafeTo(new Vector2d(36, 30))
                        .strafeTo(new Vector2d(36, 65))
                        .strafeTo(new Vector2d(-11, 4))

                        .waitSeconds(3)
                        .stopAndAdd(intake.setPower(0))
                        .stopAndAdd(shooter.setPower(0))
                        .build());


    }
}