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

@TeleOp(name = "Auto Lock Aim v2 (Clamped)", group = "Competition")
public class AutoLockAim2 extends LinearOpMode {

    // --- HARDWARE ---
    private DcMotorEx leftMotor, rightMotor, turretMotor;
    private RobotPoseController robotPoseController;

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

    private IMU imu;


    @Override
    public void runOpMode() {
        robotPoseController = new RobotPoseController(hardwareMap);
        // --- INIT HARDWARE ---
//        leftMotor = hardwareMap.get(DcMotorEx.class, "left");
//        rightMotor = hardwareMap.get(DcMotorEx.class, "right");
        turretMotor = hardwareMap.get(DcMotorEx.class, "shooterRot");

        // Motors
        turretMotor = hardwareMap.get(DcMotorEx.class, "shooterRot");
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
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // PID Safety Limit
        pidController.setOutputBounds(-1, 1);

        // Dashboard & Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // IMU Init
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        telemetry.addLine("Ready. Press Start.");
        telemetry.update();

        waitForStart();

        // Capture initial yaw to treat "Forward" as 0
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

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
            double robotYaw = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset);
            double turretTicks = turretMotor.getCurrentPosition();
            double turretDegrees = turretTicks / TICKS_PER_DEGREE;

            // Absolute Angle = Where the turret is actually pointing in the world
            double turretWorldAngle = normalizeAngle(robotYaw + turretDegrees);

            // --- 2. DRIVER CONTROL (CHASSIS) ---
//            double drive = -gamepad1.left_stick_y;
//            double turn = gamepad1.right_stick_x;
//            double leftPower = drive + turn;
//            double rightPower = drive - turn;
//
//            // Normalize power if > 1.0
//            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
//            if (max > 1.0) { leftPower /= max; rightPower /= max; }
//
//            if (Math.abs(drive) > 0.05 || Math.abs(turn) > 0.05) {
//                leftMotor.setPower(leftPower);
//                rightMotor.setPower(rightPower);
//            } else {
//                leftMotor.setPower(0);
//                rightMotor.setPower(0);
//            }

            // --- 3. INPUT HANDLING ---

            // [Square]: Set Target (Lock current heading)
            if (gamepad1.square && !isSquareClicked) {
                targetWorldAngle = turretWorldAngle;
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
                // Step A: Calculate 'Ideal' Angle relative to Robot
                // "Ideally, the turret should look X degrees to the left/right to see the target"
                double error = normalizeAngle(targetWorldAngle - robotYaw);
                double requiredTurretAngle = error;

                // Step B: Clamp to Mechanical Limits
                double clampedTargetAngle;

                if (requiredTurretAngle > MAX_TURRET_ANGLE_POSITIVE) {
                    // Robot turned too far right -> Hold Left Limit
                    clampedTargetAngle = MAX_TURRET_ANGLE_POSITIVE;
                    isAtLimit = true;
                } else if (requiredTurretAngle < MAX_TURRET_ANGLE_NEGATIVE) {
                    // Robot turned too far left -> Hold Right Limit
                    clampedTargetAngle = MAX_TURRET_ANGLE_NEGATIVE;
                    isAtLimit = true;
                } else {
                    // Within range -> Track perfectly
                    clampedTargetAngle = requiredTurretAngle;
                    isAtLimit = false;
                }

                // Step C: Execute PID
                // Convert Degrees back to Ticks for the PID controller
                double targetTicks = clampedTargetAngle * TICKS_PER_DEGREE;

                pidController.targetPosition = targetTicks;
                double power = pidController.update(turretTicks);

                turretMotor.setPower(power);

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