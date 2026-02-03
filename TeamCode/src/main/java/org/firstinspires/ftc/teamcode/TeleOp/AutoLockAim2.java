package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.controllers.RobotPoseController;
import org.firstinspires.ftc.teamcode.controllers.ShooterRotatorController;

@TeleOp(name = "Auto Lock Aim v2 (Clamped)", group = "Competition")
public class AutoLockAim2 extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotorEx leftMotor, rightMotor, turretMotor;

    // --- PID TUNING ---
    // Start with P=0.03. Increase if sluggish. D=0.001 stops oscillation.
    public static PIDCoefficients coeffs = new PIDCoefficients(0.03, 0, 0.001);
    PIDFController pidController = new PIDFController(coeffs);

    // --- CONSTANTS ---
    // Formula: (Encoder Ticks / Gear Ratio) / 360 degrees
    // Based on your code: ((256+261+265+260)/4)/90 = ~2.838 ticks per degree
    private static final double TICKS_PER_DEGREE = 2.838;

    // Mechanical Limits (Degrees relative to robot front)
    private static final double MAX_TURRET_ANGLE_POSITIVE = 170; // Left Limit
    private static final double MAX_TURRET_ANGLE_NEGATIVE = -150; // Right Limit

    // --- STATE VARIABLES ---
    private double targetWorldAngle = 0; // The compass direction we want to face (0-360)
    private boolean isAutoAimActive = false;
    private boolean isAtLimit = false;

    // Button toggles
    private boolean isCircleClicked = false;
    private boolean isSquareClicked = false;

    private double yawOffset = 0; // To reset IMU zero
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private RobotPoseController robotPoseController;
    private ShooterRotatorController shooterRotatorController;



    @Override
    public void runOpMode() {
        robotPoseController = new RobotPoseController(hardwareMap);
        shooterRotatorController = new ShooterRotatorController(hardwareMap, robotPoseController, "shooterRot");
        // --- INIT HARDWARE ---
//        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
//        rightMotor = hardwareMap.get(DcMotorEx.class, "right");

        // Motors
        frontLeft   = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight  = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft    = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight   = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        // ---------------- MOTOR DIRECTIONS ----------------
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Turret Motor


        // PID Safety Limit
        pidController.setOutputBounds(-1, 1);

        // Dashboard & Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        telemetry.addLine("Ready. Press Start.");
        telemetry.update();

        waitForStart();

        // Capture initial yaw to treat "Forward" as 0
        yawOffset = robotPoseController.getYaw();

        while (opModeIsActive()) {
            // ===== DRIVE =====
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double max = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            frontLeft.setPower((y + x + rx) / max);
            frontRight.setPower((y - x - rx) / max);
            backLeft.setPower((y - x + rx) / max);
            backRight.setPower((y + x - rx) / max);

            // --- 1. SENSOR READINGS ---
            double robotYaw = robotPoseController.getRobotYaw();

            shooterRotatorController.update();
            robotPoseController.update();
            double turretTicks = shooterRotatorController.getCurrentPosition();
            double turretDegrees = turretTicks / TICKS_PER_DEGREE;


            // --- 3. INPUT HANDLING ---

            // [Square]: Set Target (Lock current heading)
            if (gamepad1.square && !isSquareClicked) {
                shooterRotatorController.setTargetWorldAngle();
                isSquareClicked = true;
            } else if (!gamepad1.square) {
                isSquareClicked = false;
            }

            // [Circle]: Toggle Auto Aim
            if (gamepad1.circle && !isCircleClicked) {
                isAutoAimActive = !isAutoAimActive;
                isCircleClicked = true;
                // If turning off, cut power immediately
                if (!isAutoAimActive) turretMotor.setPower(0);
            } else if (!gamepad1.circle) {
                isCircleClicked = false;
            }

            // --- 4. TURRET LOGIC (CLAMPED TRACKING) ---

            if (isAutoAimActive) {
                shooterRotatorController.activate();

            } else {
                // MANUAL OVERRIDE (Gamepad 2)
                double manualPower = -gamepad2.right_stick_x * 1.0; // Half speed for manual

                // Simple Soft Limits for Manual Mode
                if (turretDegrees > MAX_TURRET_ANGLE_POSITIVE && manualPower > 0) manualPower = 0;
                if (turretDegrees < MAX_TURRET_ANGLE_NEGATIVE && manualPower < 0) manualPower = 0;

                turretMotor.setPower(manualPower);
                isAtLimit = false;
            }

            // --- 5. TELEMETRY ---
            telemetry.addData("MODE", isAutoAimActive ? "AUTO-LOCK" : "MANUAL");
            telemetry.addData("STATUS", isAtLimit ? "⚠️ LIMIT REACHED (Turn Robot!)" : "✅ TRACKING");
            telemetry.addData("Target World", "%.1f°", targetWorldAngle);
            telemetry.addData("Robot Yaw", "%.1f°", robotYaw);
            telemetry.addData("Turret Rel", "%.1f°", turretDegrees);
            telemetry.addData("Target Rel", "%.1f°", normalizeAngle(targetWorldAngle - robotYaw));
            telemetry.update();
        }
    }

    // Helper to keep angles between -180 and 180
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}