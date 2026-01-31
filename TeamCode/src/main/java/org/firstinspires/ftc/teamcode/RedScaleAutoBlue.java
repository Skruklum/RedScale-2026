package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red_Scale_Test_Auto_Blue", group = "Autonomous")
public class RedScaleAutoBlue extends LinearOpMode {

    // Set this to true for Red Alliance, false for Blue
    private boolean isBlue = true;

    public Pose2d reflect(double x, double y, double degrees) {
        return isBlue ? new Pose2d(x, -y, Math.toRadians(-degrees)) : new Pose2d(x, y, Math.toRadians(degrees));
    }

    public Vector2d reflectV(double x, double y) {
        return isBlue ? new Vector2d(x, -y) : new Vector2d(x, y);
    }

    @Override
    public void runOpMode() {
        // 1. Mirror the initial pose
        Pose2d initialPose = reflect(-70, 29, 90);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .stopAndAdd(shooter.setState(true))
                        .stopAndAdd(shooter.waitUntilReady())
                        .afterTime(0, intake.setPower(1.0))
                        // 2. Mirror every Vector2d in your strafe moves
                        .strafeTo(reflectV(-51, 10))

                        // SHOOTING PART 1
                        .stopAndAdd(stopper.timedPower(1.0))
                        .waitSeconds(2)
                        .afterTime(0, intake.setPower(0))
                        .stopAndAdd(stopper.timedPower(-1.0))

                        .strafeTo(reflectV(-20, 25))
                        .afterTime(0, intake.setPower(1))
                        .strafeTo(reflectV(-20, 65))
                        .strafeTo(reflectV(-51, 10))

                        // SHOOTING PART 2
                        .stopAndAdd(stopper.timedPower(1.0))
                        .waitSeconds(2)
                        .stopAndAdd(stopper.timedPower(-1.0))
                        .afterTime(0, intake.setPower(0))

                        .strafeTo(reflectV(28, 25))
                        .afterTime(0, intake.setPower(1))
                        .strafeTo(reflectV(28, 65))
                        .afterTime(0, intake.setPower(0))
                        .strafeTo(reflectV(-45, 4))

                        // SHOOTING PART 3
                        .afterTime(0, intake.setPower(1.0))
                        .stopAndAdd(stopper.timedPower(1.0))
                        .waitSeconds(2)
                        .build());
    }
}