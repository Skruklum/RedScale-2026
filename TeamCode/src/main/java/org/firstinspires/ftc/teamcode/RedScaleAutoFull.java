package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red/Blue Selectable Auto", group = "Autonomous")
public class RedScaleAutoFull extends LinearOpMode {

    // Selection state
    private boolean isRed = true;
    private boolean lastX = false;

    @Override
    public void runOpMode() {
        // --- PRE-START SELECTION LOOP ---
        // This runs after you hit INIT but before you hit PLAY
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !lastX) {
                isRed = !isRed; // Toggle
            }
            lastX = gamepad1.x;

            telemetry.addLine("=== ALLIANCE SELECTOR ===");
            telemetry.addData("Press X to Toggle", "Current: " + (isRed ? "RED (+)" : "BLUE (-)"));
            telemetry.addLine("------------------------");
            telemetry.addData("Status", "Waiting for Start...");
            telemetry.update();
        }

        // Logic to flip coordinates
        double side = isRed ? 1.0 : -1.0;

        // Adjust initial heading: 135 for Red, 225 for Blue
        double startHeading = isRed ? Math.toRadians(135) : Math.toRadians(225);
        Pose2d initialPose = new Pose2d(-48, 48 * side, startHeading);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeIndividual intake = new IntakeIndividual(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);

        if (isStopRequested()) return;

        // --- AUTONOMOUS PATH ---
        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .stopAndAdd(shooter.setState(true))
                        .strafeTo(new Vector2d(-12, 12 * side))
                        .stopAndAdd(intake.stagedIntake(1, 0.45))
                        .waitSeconds(0.75)
                        .stopAndAdd(shooter.setState(false))
                        .stopAndAdd(intake.setAllPower(0))
                        .stopAndAdd(stopper.setPower(1))

                        // Cycle 1
                        .strafeToLinearHeading(new Vector2d(-11, 26 * side), Math.toRadians(isRed ? 90 : 270), new TranslationalVelConstraint(45))
                        .stopAndAdd(intake.setAllPower(1))
                        .strafeToLinearHeading(new Vector2d(-11, 60 * side), Math.toRadians(isRed ? 90 : 270), new TranslationalVelConstraint(25))
                        .stopAndAdd(intake.setAllPower(0))
                        .stopAndAdd(stopper.setPower(-1))
                        .stopAndAdd(shooter.setState(true))
                        .strafeToLinearHeading(new Vector2d(-12, 12 * side), Math.toRadians(isRed ? 135 : 225), new TranslationalVelConstraint(125))
                        .stopAndAdd(intake.stagedIntake(1, 0.45))
                        .waitSeconds(0.85)
                        .stopAndAdd(shooter.setState(false))
                        .stopAndAdd(intake.setAllPower(0))
                        .stopAndAdd(stopper.setPower(1))

                        // Cycle 2
                        .strafeToLinearHeading(new Vector2d(13.18, 35 * side), Math.toRadians(isRed ? 90 : 270))
                        .stopAndAdd(intake.setAllPower(1))
                        .strafeToLinearHeading(new Vector2d(12, 62 * side), Math.toRadians(isRed ? 90 : 270), new TranslationalVelConstraint(15))
                        .stopAndAdd(intake.setAllPower(0))
                        .stopAndAdd(stopper.setPower(-1))
                        .stopAndAdd(shooter.setState(true))
                        .strafeToLinearHeading(new Vector2d(-12, 12 * side), Math.toRadians(isRed ? 135 : 225), new TranslationalVelConstraint(125))
                        .stopAndAdd(intake.stagedIntake(1, 0.45))
                        .waitSeconds(0.85)
                        .stopAndAdd(shooter.setState(false))
                        .stopAndAdd(intake.setAllPower(0))
                        .stopAndAdd(stopper.setPower(1))

                        // Cycle 3
                        .strafeToLinearHeading(new Vector2d(37, 42 * side), Math.toRadians(isRed ? 90 : 270))
                        .stopAndAdd(intake.setAllPower(1))
                        .strafeToLinearHeading(new Vector2d(37, 60 * side), Math.toRadians(isRed ? 90 : 270), new TranslationalVelConstraint(15))
                        .stopAndAdd(intake.setAllPower(0))
                        .stopAndAdd(stopper.setPower(-1))
                        .stopAndAdd(shooter.setState(true))
                        .strafeToLinearHeading(new Vector2d(-10, 14 * side), Math.toRadians(isRed ? 135 : 225), new TranslationalVelConstraint(135))
                        .stopAndAdd(intake.stagedIntake(1, 0.45))
                        .waitSeconds(0.85)
                        .stopAndAdd(shooter.setState(false))
                        .stopAndAdd(intake.setAllPower(0))
                        .stopAndAdd(stopper.setPower(1))

                        .build());
    }
}