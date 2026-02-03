package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class RsudAngleFix_AutoAimDynamicTags extends LinearOpMode {

    /* ================= HARDWARE ================= */
    private DcMotorEx shooter, turret;
    private DcMotor intake;
    private Servo shooterAngle;
    private CRServo stopper;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    /* ================= SHOOTER ================= */
    static final double HD_HEX_TICKS_PER_REV = 28.0;
    static final double TARGET_RPM = 3000.0;
    static final double SHOOTER_TICKS_PER_SEC =
            (TARGET_RPM / 60.0) * HD_HEX_TICKS_PER_REV;

    boolean shooterOn = false;
    boolean lastTrigger = false;
    boolean hasRumbled = false;

    PIDFCoefficients shooterPIDF =
            new PIDFCoefficients(20.0, 0.0, 0.0, 14.5);

    /* ================= TURRET ================= */
    static final double TICKS_PER_DEGREE = 2.838;
    static final double MAX_TURRET_POS = 170;
    static final double MAX_TURRET_NEG = -150;

    /* ================= AUTO AIM ================= */
    enum AimState { SNAP_TO_BEARING, LOCK_WORLD }
    AimState aimState = AimState.SNAP_TO_BEARING;

    /* ================= CURRENT TEAM STATE ================= */

    enum TeamState { RED, BLUE }
    TeamState teamState = TeamState.RED;

    boolean autoAim = false;
    boolean lastSquare = false;

    double yawOffset = 0;
    double targetWorldAngle = 0;
    double smoothedError = 0;

    public static double BEARING_CENTER = 1.5;
    static final double SMOOTH_ALPHA = 0.2;

    PIDFController turretPID =
            new PIDFController(new PIDCoefficients(0.015, 0, 0.7));

    /* ================= DYNAMIC TAG SWITCHING ================= */
    static final int RED_TAG = 20;
    static final int BLUE_TAG = 24;
    int currentTargetTag = RED_TAG;
    boolean lastTagButton = false;

    /* ================= VISION ================= */
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        /* ===== MAP HARDWARE ===== */
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        turret  = hardwareMap.get(DcMotorEx.class, "shooterRot");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        shooterAngle = hardwareMap.get(Servo.class, "shooterAd");
        stopper = hardwareMap.get(CRServo.class, "stopper");

        frontLeft  = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        /* ===== DIRECTIONS ===== */
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        turret.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        /* ===== ENCODERS ===== */
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF
        );

        /* ===== IMU ===== */
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();

        initVision();
        setManualExposure(6, 250);

        telemetry.addLine("READY");
        telemetry.update();

        while (true) {

            /* ===== TAG SWITCH (GAMEPAD1 A) ===== */
            if (gamepad1.a && !lastTagButton) {
                if (teamState == TeamState.RED) {
                    teamState = TeamState.BLUE;
                    currentTargetTag = BLUE_TAG;
                }else if (teamState == TeamState.BLUE) {
                    teamState = TeamState.RED;
                    currentTargetTag = RED_TAG;
                }
                aimState = AimState.SNAP_TO_BEARING;
            }
            lastTagButton = gamepad1.a;

            if (isStarted()) {
                break;
            }
        }

        waitForStart();
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (opModeIsActive()) {

            /* ===== DRIVE ===== */
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double max = Math.max(1.0,
                    Math.abs(y) + Math.abs(x) + Math.abs(rx));

            frontLeft.setPower((y + x + rx) / max);
            frontRight.setPower((y - x - rx) / max);
            backLeft.setPower((y - x + rx) / max);
            backRight.setPower((y + x - rx) / max);

            /* ===== INTAKE ===== */
            if (gamepad1.right_bumper) intake.setPower(1);
            else if (gamepad1.left_bumper) intake.setPower(-1);
            else intake.setPower(0);

            /* ===== SHOOTER ===== */
            boolean trigger = gamepad2.right_trigger > 0.2;
            if (trigger && !lastTrigger) {
                shooterOn = !shooterOn;
                hasRumbled = false;
            }
            lastTrigger = trigger;

            shooter.setVelocity(
                    shooterOn ? SHOOTER_TICKS_PER_SEC : 0
            );

            double rpm =
                    shooter.getVelocity() * 60.0 / HD_HEX_TICKS_PER_REV;

            if (shooterOn && !hasRumbled && rpm > TARGET_RPM - 50) {
                gamepad2.rumble(500);
                hasRumbled = true;
            }

            /* ===== AUTO AIM TOGGLE ===== */
            if (gamepad2.square && !lastSquare) {
                autoAim = !autoAim;
                aimState = AimState.SNAP_TO_BEARING;
            }
            lastSquare = gamepad2.square;


            updateTurretAutoAim();

            /* ===== ANGLE SERVO ===== */
            if (gamepad2.x) setServoDeg(0);
            else if (gamepad2.y) setServoDeg(45);
            else if (gamepad2.b) setServoDeg(90);

            /* ===== STOPPER ===== */
            if (gamepad2.dpad_right) stopper.setPower(1);
            else if (gamepad2.dpad_left) stopper.setPower(-1);
            else stopper.setPower(0);

            telemetry.addData("AutoAim", autoAim);
            telemetry.addData("Target Tag", currentTargetTag);
            telemetry.addData("Shooter RPM", "%.0f", rpm);
            telemetry.update();
        }

        visionPortal.close();
    }

    /* ================= AUTO AIM CORE ================= */
    private void updateTurretAutoAim() {

        double robotYaw = normalize(
                imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.DEGREES) - yawOffset
        );

        double turretTicks = turret.getCurrentPosition();
        double turretDeg = turretTicks / TICKS_PER_DEGREE;
        double turretAbs = normalize(robotYaw + turretDeg);

        if (!autoAim) {
            double manual = -gamepad2.right_stick_x * 0.6;
            if (turretDeg > MAX_TURRET_POS && manual > 0) manual = 0;
            if (turretDeg < MAX_TURRET_NEG && manual < 0) manual = 0;
            turret.setPower(manual);
            return;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean found = false;
        double bearingError = 0;

        for (AprilTagDetection d : detections) {
            if (d.metadata != null && d.id == currentTargetTag) {

                bearingError = d.ftcPose.bearing - BEARING_CENTER;
                smoothedError =
                        bearingError * SMOOTH_ALPHA +
                                smoothedError * (1 - SMOOTH_ALPHA);

                if (aimState == AimState.SNAP_TO_BEARING) {
                    targetWorldAngle =
                            normalize(turretAbs + smoothedError);
                    if (Math.abs(smoothedError) < 0.5)
                        aimState = AimState.LOCK_WORLD;
                }

                found = true;
                break;
            }
        }

        if (!found) {
            turret.setPower(0);
            return;
        }

        double errorDeg =
                normalize(targetWorldAngle - robotYaw);

        errorDeg = Range.clip(
                errorDeg, MAX_TURRET_NEG, MAX_TURRET_POS
        );

        turretPID.targetPosition =
                errorDeg * TICKS_PER_DEGREE;

        double power =
                turretPID.update(turretTicks);

        turret.setPower(Range.clip(power, -1, 1));
    }

    /* ================= HELPERS ================= */
    private void setServoDeg(double d) {
        shooterAngle.setPosition(Range.clip(d / 180.0, 0, 1));
    }

    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(
                        new Position(DistanceUnit.CM, 0, 6, 43, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)
                )
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    private void setManualExposure(int exp, int gain) {
        ExposureControl e =
                visionPortal.getCameraControl(ExposureControl.class);
        GainControl g =
                visionPortal.getCameraControl(GainControl.class);
        e.setMode(ExposureControl.Mode.Manual);
        e.setExposure(exp, TimeUnit.MILLISECONDS);
        g.setGain(gain);
    }
}
