package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class RsudAngleFixIniYangBenerCoHold2 extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotorEx turret;
    private Servo degree;
    private CRServo stopperS;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // ---------------- CONSTANTS ----------------
    // HD Hex (No Gearbox) = 28 ticks per rev
    static final double HD_HEX_TICKS_PER_REV = 28.0;

    // Shooter Constants (KEPT AS REQUESTED)
    static final double TARGET_RPM = 2950.0;
    static final double SHOOTER_TICKS_PER_SEC = (TARGET_RPM / 60.0) * HD_HEX_TICKS_PER_REV;

    // Turret Constants
    static final double TURRET_POWER = 0.8; // Speed for manual control
    // ---------------- STATE ----------------
    boolean shooterOn = false;
    boolean lastTrigger = false;
    boolean hasRumbled = false;

    // (SHOOTER PID - UNTOUCHED)
    public static double PID_P = 78.0;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0;
    public static double PID_F = 16.12222;
    PIDFCoefficients ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = hardwareMap.get(DcMotorEx.class, "shooterRot");
        degree = hardwareMap.get(Servo.class, "shooterAd");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        stopperS = hardwareMap.get(CRServo.class, "stopper");

        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )));

        // ---------------- MOTOR DIRECTIONS ----------------
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // ---------------- ENCODER SETUP ----------------
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: Shooter must be in RUN_USING_ENCODER for RPM limiting to work
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
        // SHOOTER FLOAT BEHAVIOR (KEPT)
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Ready to Start (Manual Turret - NO LIMITS)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check if coefficients need updating (add this inside the while loop)


            if (PID_F != ShooterPIDF.f || PID_P != ShooterPIDF.p) {
                ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
                shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
            }

            // ===== INTAKE =====
            if (gamepad1.right_bumper) intake.setPower(1);
            else if (gamepad1.left_bumper) intake.setPower(-1);
            else intake.setPower(0);

            // ===== SHOOTER TOGGLE (KEPT) =====
            boolean trigger = gamepad2.right_trigger > 0.2;
            if (trigger && !lastTrigger) {
                shooterOn = !shooterOn;
                hasRumbled = false;
            }
            lastTrigger = trigger;

            if (shooterOn) {
                shooter.setVelocity(SHOOTER_TICKS_PER_SEC);
            } else {
                shooter.setVelocity(0);
            }

            // ===== SHOOTER ANGLE SERVO =====
            if (gamepad2.a) setServoDegrees(0);
            else if (gamepad2.b) setServoDegrees(45);
            else if (gamepad2.y) setServoDegrees(90);

            // ===== STOPPER =====
            if (gamepad2.dpad_right) stopperS.setPower(1);
            else if (gamepad2.dpad_left) stopperS.setPower(-1);
            else stopperS.setPower(0);

            // ===== DRIVE =====
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double max = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            frontLeft.setPower((y + x + rx) / max);
            frontRight.setPower((y - x - rx) / max);
            backLeft.setPower((y - x + rx) / max);
            backRight.setPower((y + x - rx) / max);

            // =================================================================
            //                        MANUAL TURRET LOGIC
            // =================================================================

            double manualTurretPower = 0;

            if (gamepad2.right_bumper) {
                manualTurretPower = -TURRET_POWER;
            } else if (gamepad2.left_bumper) {
                manualTurretPower = TURRET_POWER;
            } else if (Math.abs(gamepad2.right_stick_x) > 0.05) {
                // Analog control using stick
                manualTurretPower = -gamepad2.right_stick_x * 0.6;
            }

            // Direct power set - NO LIMITERS, NO PID
            turret.setPower(manualTurretPower);


            // ===== TELEMETRY & RUMBLE =====
            double currentRPM = (shooter.getVelocity() * 60.0) / HD_HEX_TICKS_PER_REV;

            // Rumble when we hit the target RPM (+/- 50 RPM)
            if (shooterOn && !hasRumbled && currentRPM >= (TARGET_RPM - 50)) {
                gamepad2.rumble(500);
                hasRumbled = true;
            }

            telemetry.addData("Shooter Mode", shooterOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Turret Power", "%.2f", manualTurretPower);
            telemetry.update();
        }
    }

    private void setServoDegrees(double deg) {
        degree.setPosition(Math.min(1.0, Math.max(0.0, deg / 180.0)));
    }
}