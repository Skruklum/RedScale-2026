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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class RsudAngleFixIniYangBenerCoHold2 extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotor intake, intake2, shooterTop, shooterBottom;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private CRServo leftServo;
    private Servo shooterServo;
    private Servo angleServo;
    private CRServo stopperServo;
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

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        shooterTop = hardwareMap.get(DcMotor.class,"shooter1");
        shooterBottom = hardwareMap.get(DcMotor.class, "shooter2");

        leftServo = hardwareMap.get(CRServo.class, "lServo");
        shooterServo = hardwareMap.get(Servo.class, "sServo");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        stopperServo = hardwareMap.get(CRServo.class, "stopServo");

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
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.FORWARD);

        shooterTop.setDirection(DcMotor.Direction.FORWARD);
        shooterBottom.setDirection(DcMotor.Direction.REVERSE);

        // ---------------- ENCODER SETUP ----------------

        // IMPORTANT: Shooter must be in RUN_USING_ENCODER for RPM limiting to work
        // SHOOTER FLOAT BEHAVIOR (KEPT)

        telemetry.addLine("Ready to Start (Manual Turret - NO LIMITS)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check if coefficients need updating (add this inside the while loop)

            if (PID_F != ShooterPIDF.f || PID_P != ShooterPIDF.p) {
                ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
//                shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
            }

            // ===== FRONT INTAKE =====
            if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // ===== BACK INTAKE =====
            if (gamepad1.right_trigger > 0){
                intake2.setPower(1);
            } else if (gamepad1.left_trigger > 0) {
                intake2.setPower(-1);
            } else {
                intake2.setPower(0.0) ;
            }

            // ===== SHOOTER TOGGLE =====
            boolean trigger = gamepad2.right_trigger > 0.2;
            if (trigger && !lastTrigger) shooterOn = !shooterOn;
            lastTrigger = trigger;
            shooterTop.setPower(shooterOn ? 1.0 : 0.0);
            shooterBottom.setPower(shooterOn ? 1.0 : 0.0);

            // ===== TURRET MANUAL =====
            if (gamepad2.right_bumper) leftServo.setPower(-1);
            else if (gamepad2.left_bumper) leftServo.setPower(1);
            else leftServo.setPower(0);
            // ===== STOPPER MANUAL =====
            if (gamepad2.dpad_right){
                stopperServo.setPower(-1.0);
            } else if (gamepad2.dpad_left) {
                stopperServo.setPower(1.0);
            } else {
                stopperServo.setPower(0.0);
            }

            // ===== SHOOTER ANGLE SERVO =====
            if (gamepad2.x) angleServo.setPosition(0);
            else if (gamepad2.y) angleServo.setPosition(5);
            else if (gamepad2.b) angleServo.setPosition(45);

//            if (gamepad1.a) frontLeft.setPower(1);
//            else if (gamepad1.b) frontRight.setPower(1);
//            else if (gamepad1.x) backLeft.setPower(1);
//            else if (gamepad1.y) backRight.setPower(1);
//            else {
//                frontRight.setPower(0);
//                frontLeft.setPower(0);
//                backRight.setPower(0);
//                backLeft.setPower(0);
//            }

            // ===== DRIVE =====
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double max = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            frontLeft.setPower((y + x + rx) / max);
            frontRight.setPower((y - x - rx) / max);
            backLeft.setPower((y - x + rx) / max);
            backRight.setPower((y + x - rx) / max);

            telemetry.update();
        }
    }

    // ---------------- FUNCTIONS ----------------

    private void setServoDegrees(double deg) {
        shooterServo.setPosition(Math.min(1.0, Math.max(0.0, deg / 180.0)));
    }
}