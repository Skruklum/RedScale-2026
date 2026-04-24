package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp(name = "MainTeleOp", group = "TeleOp")
public class MergedTeleOp extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotor intake, intake2;
    private DcMotorEx shooterTop, shooterBottom;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private CRServo leftServo;
    private Servo shooterServo;
    private IMU  imu;

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

    // ---------------- SHOOTER PIDF ----------------
    // F = 32767 / SHOOTER_TICKS_PER_SEC = 32767 / 2800 = ~11.702
    // P increased for fast velocity recovery at 6000 RPM
    public static double PID_P = 90.0;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0;
    public static double PID_F = 11.702;
    PIDFCoefficients ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);

    // ================== PID (AprilTag) ==================
    public static PIDCoefficients pidVision = new PIDCoefficients(0.015, 0, 0.7);
    public static PIDCoefficients pidGyro   = new PIDCoefficients(0.035, 0, 0.001);

    // ================== TURRET ==================
    private static final double MAX_TURRET_ANGLE_POSITIVE = 170;
    private static final double MAX_TURRET_ANGLE_NEGATIVE = -150;

    /**
     * Ticks-per-degree for the odometry wheel attached to the turret gear.
     * Tune this to match your odometry pod's resolution and gear ratio.
     */
    public static double TICKS_PER_DEGREE = 2.838;

    // ================== VISION ==================
    public static double BEARING_CENTER = 1.5;
    private static final double SMOOTHING_ALPHA = 0.2;

    // ================== CAMERA ==================
    private static final long EXPOSURE_MS = 6;
    private static final int CAMERA_GAIN = 250;
    private static final float DECIMATION_SEARCH = 2.0f;

    public static int RED_GOAL_TAG_ID = 20;

    enum AimState {
        SNAP_TO_BEARING,
        LOCK_WORLD
    }

    private AimState aimState = AimState.SNAP_TO_BEARING;

    // ================== HARDWARE (AprilTag) ==================
    /**
     * Turret is now a continuous-rotation servo (CRServo).
     * Map it to "shooterRot" in your hardware config.
     */
    private CRServo turretServo;

    /**
     * The odometry pod tracking turret rotation is plugged into the
     * "BackIntake" motor port. We only read its encoder — never set power.
     */
    private DcMotorEx turretOdometry;

    // ================== VISION HARDWARE ==================
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final CameraStreamProcessor streamProcessor = new CameraStreamProcessor();

    // ================== STATE (AprilTag) ==================
    private PIDFController turretPID = new PIDFController(pidGyro);
    private boolean usingVisionGains = false;

    private boolean isAutoAim = false;
    private boolean lastSquare = false;
    private boolean isAtLimit = false;

    private double yawOffset = 0;

    /**
     * The world-angle we want the turret to point at.
     * Updated whenever the AprilTag is visible; held at the last value when lost.
     */
    private double targetWorldAngle = 0;

    /** Whether we have ever seen the tag and thus have a valid last-known angle. */
    private boolean hasLastKnownTarget = false;

    private double smoothedBearingError = 0;

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

        frontLeft  = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )));

        // --- Turret: CRServo for motion, separate odometry for position ---
        turretServo    = hardwareMap.get(CRServo.class,  "shooterRot");
        turretOdometry = hardwareMap.get(DcMotorEx.class, "BackIntake");

        // Reset and configure the odometry motor (encoder-only, no power output)
        turretOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretOdometry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Do NOT set power on turretOdometry — it's only used for encoder reads.

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

        turretPID.setOutputBounds(-1.0, 1.0);

        // ---------------- VISION INIT ----------------
        initVision();

        telemetry.addLine("Ready to Start");
        telemetry.update();

        waitForStart();

        // ================== POST-START SETUP ==================
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        setManualExposure(EXPOSURE_MS, CAMERA_GAIN);

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

            // ===== TURRET MANUAL (gamepad2 bumpers) =====
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

            // ===== AUTO AIM TOGGLE =====
            if (gamepad2.square && !lastSquare) {
                isAutoAim = !isAutoAim;
                aimState  = AimState.SNAP_TO_BEARING;
            }
            lastSquare = gamepad2.square;

            // ===== SENSOR READS =====
            double robotYaw = normalizeAngle(
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset
            );

            // Turret position from odometry pod on BackIntake port
            double turretTicks = turretOdometry.getCurrentPosition();
            double turretDeg   = turretTicks / TICKS_PER_DEGREE;
            double turretAbs   = normalizeAngle(robotYaw + turretDeg);

            // ===== AUTO AIM =====
            boolean tagVisible   = false;
            double  rawBearing   = 0;
            double  bearingError = 0;

            if (isAutoAim) {
                List<AprilTagDetection> detections = aprilTag.getDetections();

                for (AprilTagDetection d : detections) {
                    telemetry.addData("DETECTED ID", d.metadata != null ? d.metadata.id : d.id);

                    if (d.metadata != null && d.id == RED_GOAL_TAG_ID) {
                        tagVisible   = true;
                        rawBearing   = d.ftcPose.bearing;
                        bearingError = rawBearing - BEARING_CENTER;

                        smoothedBearingError =
                                bearingError         * SMOOTHING_ALPHA +
                                        smoothedBearingError * (1.0 - SMOOTHING_ALPHA);

                        // --- Update target from live tag data ---
                        if (aimState == AimState.SNAP_TO_BEARING) {
                            targetWorldAngle = normalizeAngle(turretAbs + smoothedBearingError);
                            if (Math.abs(smoothedBearingError) < 0.5) {
                                aimState = AimState.LOCK_WORLD;
                            }
                        } else {
                            // In LOCK_WORLD, keep refreshing so we track movement
                            targetWorldAngle = normalizeAngle(turretAbs + smoothedBearingError);
                        }

                        hasLastKnownTarget = true; // we now have a valid world angle saved

                        if (!usingVisionGains) {
                            turretPID = new PIDFController(pidVision);
                            usingVisionGains = true;
                        }
                        break;
                    }
                }

                /*
                 * If the tag is NOT visible but we have a last-known world angle,
                 * continue driving the turret toward that saved angle.
                 * If we have never seen the tag, hold position (power = 0).
                 */
                if (!tagVisible && !hasLastKnownTarget) {
                    turretServo.setPower(0);
                    isAtLimit = false;
                } else {
                    // Drive toward targetWorldAngle (either live or last-known)
                    double errorDeg = normalizeAngle(targetWorldAngle - robotYaw);
                    errorDeg = Range.clip(
                            errorDeg,
                            MAX_TURRET_ANGLE_NEGATIVE,
                            MAX_TURRET_ANGLE_POSITIVE
                    );

                    turretPID.targetPosition = errorDeg * TICKS_PER_DEGREE;
                    double power = turretPID.update(turretTicks);

                    // Hard limits
                    if (turretDeg > MAX_TURRET_ANGLE_POSITIVE && power > 0) {
                        power = 0;
                        isAtLimit = true;
                    } else if (turretDeg < MAX_TURRET_ANGLE_NEGATIVE && power < 0) {
                        power = 0;
                        isAtLimit = true;
                    } else {
                        isAtLimit = false;
                    }

                    turretServo.setPower(Range.clip(power, -1.0, 1.0));
                }

            } else {
                /* ===== MANUAL TURRET (right stick) ===== */
                double manual = -gamepad2.right_stick_x * 0.6;

                if (turretDeg > MAX_TURRET_ANGLE_POSITIVE && manual > 0) manual = 0;
                if (turretDeg < MAX_TURRET_ANGLE_NEGATIVE && manual < 0) manual = 0;

                turretServo.setPower(manual);
                isAtLimit = false;

                // Keep last-known angle in sync with where the turret actually is
                // so when auto-aim re-engages it doesn't snap wildly
                targetWorldAngle = turretAbs;
            }

            // ===== TELEMETRY =====
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Target Ticks/s", SHOOTER_TICKS_PER_SEC);
            telemetry.addData("Top Velocity (ticks/s)",    shooterTop.getVelocity());
            telemetry.addData("Bottom Velocity (ticks/s)", shooterBottom.getVelocity());
            telemetry.addData("Top RPM",    (shooterTop.getVelocity() / HD_HEX_TICKS_PER_REV) * 60.0);
            telemetry.addData("Bottom RPM", (shooterBottom.getVelocity() / HD_HEX_TICKS_PER_REV) * 60.0);

            telemetry.addLine("===== AUTO AIM DEBUG =====");
            telemetry.addData("AutoAim",          isAutoAim);
            telemetry.addData("AimState",         aimState);
            telemetry.addData("AtLimit",          isAtLimit);
            telemetry.addData("HasLastKnownTgt",  hasLastKnownTarget);

            telemetry.addLine("----- ANGLES -----");
            telemetry.addData("RobotYaw",    robotYaw);
            telemetry.addData("TurretDeg",   turretDeg);
            telemetry.addData("TurretAbs",   turretAbs);
            telemetry.addData("TargetWorld", targetWorldAngle);

            telemetry.addLine("----- ODOMETRY -----");
            telemetry.addData("TurretTicks (BackIntake)", turretTicks);

            telemetry.addLine("----- VISION -----");
            telemetry.addData("TagVisible",    tagVisible);
            telemetry.addData("RawBearing",    rawBearing);
            telemetry.addData("BearingCenter", BEARING_CENTER);
            telemetry.addData("BearingError",  bearingError);
            telemetry.addData("SmoothedError", smoothedBearingError);

            telemetry.addLine("----- PID -----");
            telemetry.addData("PID Mode",           usingVisionGains ? "VISION" : "GYRO");
            telemetry.addData("PID Target (ticks)",  turretPID.targetPosition);

            telemetry.update();
        }

        visionPortal.close();
    }

    // ---------------- FUNCTIONS ----------------

    /**
     * Anti-Clog: runs both intakes forward and spins shooter motors at -0.3 raw power
     * to break up any jammed rings. Triggered by gamepad2 left_trigger.
     */
    private void antiClogIntake() {
        intake.setPower(1);
        intake2.setPower(0.69);
        // Raw power mode needed for -0.3 (velocity PID can't command negative)
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

    // ================== UTILS ==================
    private double normalizeAngle(double a) {
        while (a >  180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private void setManualExposure(long exposureMS, int gain) {
        ExposureControl exposure    = visionPortal.getCameraControl(ExposureControl.class);
        GainControl     gainControl = visionPortal.getCameraControl(GainControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        gainControl.setGain(gain);
    }

    private void initVision() {
        Position           camPos = new Position(DistanceUnit.CM, 0, 6, 43, 0);
        YawPitchRollAngles camRot = new YawPitchRollAngles(
                AngleUnit.DEGREES, 0, 0, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(camPos, camRot)
                .setDrawTagID(true)
                .build();

        aprilTag.setDecimation(DECIMATION_SEARCH);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(streamProcessor)
                .setCameraResolution(new Size(640, 480))
                .build();

        FtcDashboard.getInstance().startCameraStream(streamProcessor, 0);
    }

    // ================== CAMERA STREAM ==================
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        public void init(int w, int h, CameraCalibration c) {
            lastFrame.set(Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565));
        }

        public Object processFrame(Mat frame, long t) {
            Bitmap b = lastFrame.get();
            if (b.getWidth() != frame.width() || b.getHeight() != frame.height()) {
                b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
                lastFrame.set(b);
            }
            Utils.matToBitmap(frame, b);
            return null;
        }

        public void onDrawFrame(Canvas c, int w, int h, float s1, float s2, Object o) {}

        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> cont) {
            cont.dispatch(bc -> bc.accept(lastFrame.get()));
        }
    }
}