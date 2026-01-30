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
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        // Build and execute the action
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .stopAndAdd(shooter.setState(true))
                        .stopAndAdd(shooter.waitUntilReady())
                        .afterTime(0, intake.setPower(1.0))
                        .strafeTo(new Vector2d(-51, 10))


                        //SHOOTING PART 1
                        .stopAndAdd(stopper.timedPower(1.0))
                        .waitSeconds(2)
                        .afterTime(0, intake.setPower(0))
                        .stopAndAdd(stopper.timedPower(-1.0))
                        //


                        .strafeTo(new Vector2d(-20  , 25))
                        .afterTime(0, intake.setPower(1))
                        .strafeTo(new Vector2d(-20, 65))
                        .strafeTo(new Vector2d(-51, 10))


                        //SHOOTING PART 2
                        .stopAndAdd(stopper.timedPower(1.0))
                        .waitSeconds(2)
                        .stopAndAdd(stopper.timedPower(-1.0))
                        .afterTime(0, intake.setPower(0))
                        //



                        //.splineTo(new Vector2d(37.5, 60), Math.toRadians(90))
                        .strafeTo(new Vector2d(28, 25))
                        .afterTime(0, intake.setPower(1))
                        .strafeTo(new Vector2d(28, 65))
                        .afterTime(0, intake.setPower(0))
                        .strafeTo(new Vector2d(-45, 4))


                        //SHOOTING PART 3
                        .afterTime(0, intake.setPower(1.0))
                        .stopAndAdd(stopper.timedPower(1.0))
                        .waitSeconds(2)
                        //

                        .build());


    }
}