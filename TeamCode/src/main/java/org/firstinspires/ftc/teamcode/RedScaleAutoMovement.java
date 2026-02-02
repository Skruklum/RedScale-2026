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
                        .strafeTo(new Vector2d(-51, 10))
                        .waitSeconds(3)
                        .strafeTo(new Vector2d(-20  , 25))
                        .strafeTo(new Vector2d(-20, 65))
                      //  .splineTo(new Vector2d(12, 60), Math.toRadians(90))
                        .strafeTo(new Vector2d(-51, 10))
                        .waitSeconds(3)
                        //.splineTo(new Vector2d(37.5, 60), Math.toRadians(90))
                        .strafeTo(new Vector2d(28, 25))
                        .strafeTo(new Vector2d(28, 65))
                        .strafeTo(new Vector2d(-51, 10))


                        .waitSeconds(3)

                        .build());


    }
}