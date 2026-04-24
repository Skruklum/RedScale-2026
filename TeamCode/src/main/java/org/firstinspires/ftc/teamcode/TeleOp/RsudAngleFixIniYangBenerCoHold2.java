package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class RsudAngleFixIniYangBenerCoHold2 extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotor intake, intake2;
    private DcMotorEx shooterTop, shooterBottom;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private CRServo leftServo;
    private Servo shooterServo;
    private IMU imu;
    private ColorSensor colorSensor;
    private CRServo stopperServo;
    // ---------------- CONSTANTS ----------------
    // HD Hex (No Gearbox) = 28 ticks per rev
    static final double HD_HEX_TICKS_PER_REV = 28.0;

    // Shooter Constants — Ultra Planetary HD Hex max = 6000 RPM
    static final double TARGET_RPM = 6000.0;
    // Ticks/sec = (6000 / 60) * 28 = 2800
    static final double SHOOTER_TICKS_PER_SEC = (TARGET_RPM / 60.0) * HD_HEX_TICKS_PER_REV;

    // Turret Constants
    static final double TURRET_POWER = 0.8;

    // ---------------- STATE ----------------
    boolean shooterOn = false;
    boolean lastTrigger = false;
    boolean hasRumbled = false;

    // ---------------- COLOR SENSOR STATE ----------------
    private long colorDetectStartTime = 0;
    private boolean colorDetecting = false;

    // ---------------- SHOOTER PIDF ----------------
    // F = 32767 / SHOOTER_TICKS_PER_SEC = 32767 / 2800 = ~11.702
    // P increased for fast velocity recovery at 6000 RPM
    public static double PID_P = 90.0;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0;
    public static double PID_F = 11.702;
    PIDFCoefficients ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake  = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        shooterTop    = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooter2");

        leftServo    = hardwareMap.get(CRServo.class, "lServo");
        shooterServo = hardwareMap.get(Servo.class, "sServo");
        stopperServo = hardwareMap.get(CRServo.class, "stopServo");

        frontLeft  = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )));

        // ---------------- MOTOR DIRECTIONS ----------------
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.FORWARD);

        shooterTop.setDirection(DcMotor.Direction.FORWARD);
        shooterBottom.setDirection(DcMotor.Direction.REVERSE);

        // ---------------- SHOOTER ENCODER + PID SETUP ----------------
        shooterTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
        shooterBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);

        shooterTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== LIVE PIDF TUNING via FTC Dashboard =====
            if (PID_F != ShooterPIDF.f || PID_P != ShooterPIDF.p
                    || PID_I != ShooterPIDF.i || PID_D != ShooterPIDF.d) {
                ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
                shooterTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
                shooterBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
            }

            // ===== FRONT INTAKE =====
            if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // ===== STOPPER =====

            if (gamepad2.dpad_right) {
                stopperServo.setPower(1);
            } else if (gamepad2.dpad_left) {
                stopperServo.setPower(-1);
            } else {
                stopperServo.setPower(0);
            }


            // ===== BACK INTAKE =====
            if (gamepad1.right_trigger > 0) {
                intake2.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                intake2.setPower(-1);
            } else {
                intake2.setPower(0.0);
            }

            // ===== ANTI-CLOG (gamepad2 left trigger) =====
            if (gamepad2.left_trigger > 0.2) {
                antiClogIntake();
            }
            // ===== SHOOTER TOGGLE (gamepad2 right trigger) =====
            else {
                boolean trigger = gamepad2.right_trigger > 0.2;
                if (trigger && !lastTrigger) shooterOn = !shooterOn;
                lastTrigger = trigger;

                if (shooterOn) {
                    // Restore PID mode in case anti-clog switched it away
                    if (shooterTop.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        shooterTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        shooterBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        shooterTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
                        shooterBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
                    }
                    // setVelocity commands the PID to hold exactly TARGET_RPM
                    shooterTop.setVelocity(SHOOTER_TICKS_PER_SEC);
                    shooterBottom.setVelocity(SHOOTER_TICKS_PER_SEC);
                } else {
                    shooterTop.setPower(0.0);
                    shooterBottom.setPower(0.0);
                }
            }

            // ===== TURRET MANUAL =====
            if (gamepad2.right_bumper) {
                leftServo.setPower(-1.0);
            } else if (gamepad2.left_bumper) {
                leftServo.setPower(1.0);
            } else {
                leftServo.setPower(0.0);
            }

            // ===== SHOOTER ANGLE SERVO =====
            if (gamepad2.x)      setServoDegrees(0);
            else if (gamepad2.y) setServoDegrees(45);
            else if (gamepad2.b) setServoDegrees(90);

            // ===== DRIVE =====
            double y  = -gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double max = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            frontLeft.setPower((y + x + rx) / max);
            frontRight.setPower((y - x - rx) / max);
            backLeft.setPower((y - x + rx) / max);
            backRight.setPower((y + x - rx) / max);

            // ===== COLOR SENSOR CHECK =====
            checkColorAndRumble();

            // ===== TELEMETRY =====
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Target Ticks/s", SHOOTER_TICKS_PER_SEC);
            telemetry.addData("Top Velocity (ticks/s)",    shooterTop.getVelocity());
            telemetry.addData("Bottom Velocity (ticks/s)", shooterBottom.getVelocity());
            telemetry.addData("Top RPM",    (shooterTop.getVelocity() / HD_HEX_TICKS_PER_REV) * 60.0);
            telemetry.addData("Bottom RPM", (shooterBottom.getVelocity() / HD_HEX_TICKS_PER_REV) * 60.0);
            telemetry.addData("Color R", colorSensor.red());
            telemetry.addData("Color G", colorSensor.green());
            telemetry.addData("Color B", colorSensor.blue());
            telemetry.addData("Color Detecting", colorDetecting);
            telemetry.update();
        }
    }

    // ---------------- FUNCTIONS ----------------

    /**
     * Checks if the Color Sensor v3 detects purple or green for 2 continuous seconds
     * using ratio-based detection for reliability across different distances and lighting.
     * If so, vibrates gamepad1 for 2 seconds.
     */
    private void checkColorAndRumble() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int total = r + g + b;

        // Avoid division by zero if sensor reads nothing
        if (total < 30) {
            colorDetecting = false;
            colorDetectStartTime = 0;
            return;
        }

        // Normalize to ratios (0.0 – 1.0)
        double rRatio = (double) r / total;
        double gRatio = (double) g / total;
        double bRatio = (double) b / total;

        // Purple: red and blue are both dominant, green is weak
        boolean isPurple = (rRatio > 0.25 && bRatio > 0.25 && gRatio < 0.30);

        // Green: green is clearly dominant
        boolean isGreen  = (gRatio > 0.40 && rRatio < 0.35 && bRatio < 0.30);

        if (isPurple || isGreen) {
            if (!colorDetecting) {
                // Start the timer on first detection
                colorDetectStartTime = System.currentTimeMillis();
                colorDetecting = true;
            } else if (System.currentTimeMillis() - colorDetectStartTime >= 2000) {
                // Held for 2 seconds — rumble gamepad1 for 2 seconds
                gamepad1.rumble(1.0, 1.0, 2000);
                // Reset so it doesn't spam rumble every loop
                colorDetecting = false;
                colorDetectStartTime = 0;
            }
        } else {
            // Color lost — reset the timer
            colorDetecting = false;
            colorDetectStartTime = 0;
        }

        // Debug telemetry — use these to fine-tune thresholds
        telemetry.addData("rRatio", String.format("%.2f", rRatio));
        telemetry.addData("gRatio", String.format("%.2f", gRatio));
        telemetry.addData("bRatio", String.format("%.2f", bRatio));
        telemetry.addData("isPurple", isPurple);
        telemetry.addData("isGreen",  isGreen);
    }

    /**
     * Anti-Clog: runs both intakes forward and spins shooter motors at -0.7 raw power
     * to break up any jammed rings. Triggered by gamepad2 left_trigger.
     */
    private void antiClogIntake() {
        intake.setPower(1);
        intake2.setPower(-1);
        // Raw power mode needed for negative power (velocity PID can't command negative)
        shooterTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTop.setPower(-0.7);
        shooterBottom.setPower(-0.7);
    }

    /**
     * Sets the shooter angle servo position from a degree value (0–180).
     */
    private void setServoDegrees(double deg) {
        shooterServo.setPosition(Math.min(1.0, Math.max(0.0, deg / 180.0)));
    }
}