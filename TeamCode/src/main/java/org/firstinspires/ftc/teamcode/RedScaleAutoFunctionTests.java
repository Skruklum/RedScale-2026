package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Function yah", group = "Autonomous")
public class RedScaleAutoFunctionTests extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-10, 30, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        IntakeIndividual intake = new IntakeIndividual(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);
        Shooter shooter= new Shooter(hardwareMap);


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .stopAndAdd(shooter.setState(true))
                        .waitSeconds(1.5)
                        .stopAndAdd(intake.stagedIntake(1, 0.5))
                        .waitSeconds(3)

                        .build());
    }
}