package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red_Scale_Test_Auto_Function_Tests", group = "Autonomous")
public class RedScaleAutoFunctionTests extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize your MecanumDrive (this contains your 2-dead wheel localizer)
        // Make sure the starting Pose matches your MeepMeep code exactly
        Pose2d initialPose = new Pose2d(-70, 29, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        // Build and execute the action
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
//                        .afterTime(0, intake.setPower(1.0))
//                        .afterTime(0, shooter.setPower(1.0))
//                        .waitSeconds(3)
//                        .stopAndAdd(intake.setPower(0))
//                        .stopAndAdd(shooter.setPower(0))
                        .afterTime(0, stopper.setPower(1.0))
                        .waitSeconds(3)
                        .afterTime(0, stopper.setPower(-1.0))
                        .waitSeconds(3)
                        .stopAndAdd(shooter.setPower(0))
                        .build());


    }
}