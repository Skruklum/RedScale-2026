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
@TeleOp(name = "RsudAngleFix_Merged_RedAlliance", group = "TeleOp") public class RsudAngleFixIniYangBenerCoHold2 extends LinearOpMode {

// =========================================================================
//                            FILE 1 VARIABLES
// =========================================================================

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
    static final double TURRET_GEAR_RATIO = 4.75;
    static final double CORE_HEX_TICKS_PER_REV = 288.0 * TURRET_GEAR_RATIO;

    // Shooter Constants (UPDATED TO MATCH SHOOTER CLASS)
    static final double TARGET_RPM = 2950.0;
    static final double SHOOTER_TICKS_PER_SEC = (TARGET_RPM / 60.0) * HD_HEX_TICKS_PER_REV;

    // Turret Constants
    static final double TURRET_POWER = 0.6; // Speed for manual control
    static final double TURRET_LIMIT_DEG = 60.0;
    static final int TURRET_LIMIT_TICKS = (int) ((TURRET_LIMIT_DEG / 360.0) * CORE_HEX_TICKS_PER_REV);

    // ---------------- STATE ----------------
    boolean shooterOn = false;
    boolean lastTrigger = false;
    boolean hasRumbled = false;

    // (UPDATED PIDS TO MATCH SHOOTER CLASS)
    public static double PID_P = 60.0;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0;
    public static double PID_F = 17.5;
    PIDFCoefficients ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);

// =========================================================================
//                            FILE 2 VARIABLES
// =========================================================================

    /* ================== PID ================== */
    // PID for when we see the tag (needs to be responsive)
    public static PIDCoefficients pidVision = new PIDCoefficients(0.01, 0, 0.001);
    // PID for holding position when tag is lost (needs to be stiff)
    public static PIDCoefficients pidGyro   = new PIDCoefficients(0.035, 0, 0.002);

    /* ================== TURRET SETTINGS (FILE 2) ================== */
    private static final double MAX_TURRET_ANGLE_POSITIVE = 170;
    private static final double MAX_TURRET_ANGLE_NEGATIVE = -150;
    private static final double TICKS_PER_DEGREE = 2.838; // Check this calibration!

    /* ================== VISION SETTINGS ================== */
    public static double BEARING_CENTER = 1.5; // Offset if camera isn't perfectly centered
    // Alpha 0.6 means we trust new data 60%, old data 40%. Higher = Faster, Lower = Smoother
    private static final double SMOOTHING_ALPHA = 0.6;

    /* ================== CAMERA SETTINGS ================== */
    private static final long EXPOSURE_MS = 6;
    private static final int CAMERA_GAIN = 250;
    private static final float DECIMATION_SEARCH = 2.0f;

    public static int RED_GOAL_TAG_ID = 24;

    /* ================== HARDWARE ALIAS ================== */
    private DcMotorEx turretMotor; // Will point to 'turret'

    /* ================== VISION OBJECTS ================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final CameraStreamProcessor streamProcessor = new CameraStreamProcessor();

    /* ================== STATE VARIABLES ================== */
    private PIDFController turretPID = new PIDFController(pidGyro);
    private boolean usingVisionGains = false;

    private boolean isAutoAim = false;
    private boolean lastSquare = false;
    private boolean isAtLimit = false;

    private double yawOffset = 0;
    private double targetWorldAngle = 0; // The "World Heading" we want to face
    private double smoothedBearingError = 0;


    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP (FILE 1 & 2 MERGED) ----------------
        // Setup Telemetry to Dashboard and Driver Station
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = hardwareMap.get(DcMotorEx.class, "shooterRot");
        turretMotor = turret; // ALIAS for File 2 Logic

        degree = hardwareMap.get(Servo.class, "shooterAd");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        stopperS = hardwareMap.get(CRServo.class, "stopper");

        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        imu = hardwareMap.get(IMU.class, "imu");
        // Using File 1 IMU params as they seem more specific to the robot config
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
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // From File 2

        turretPID.setOutputBounds(-1.0, 1.0); // From File 2

        // IMPORTANT: Shooter must be in RUN_USING_ENCODER for RPM limiting to work
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);

        // (UPDATED LOGIC: ADDED FLOAT BEHAVIOR FROM SHOOTER CLASS)
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // ---------------- VISION INIT (FILE 2) ----------------
        initVision();

        telemetry.addLine("Ready to Start (Manual Turret Mode)");
        telemetry.update();

        waitForStart();

        // ---------------- START (FILE 2 Logic) ----------------
        // Set the zero-yaw offset
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        setManualExposure(EXPOSURE_MS, CAMERA_GAIN);

        while (opModeIsActive()) {

            // =================================================================
            //                        FILE 1 MECHANISM LOGIC
            // =================================================================

            // ===== INTAKE =====
            if (gamepad1.right_bumper) intake.setPower(1);
            else if (gamepad1.left_bumper) intake.setPower(-1);
            else intake.setPower(0);

            // ===== SHOOTER TOGGLE (UPDATED 2950 RPM LIMIT) =====
            boolean trigger = gamepad2.right_trigger > 0.2;
            if (trigger && !lastTrigger) {
                shooterOn = !shooterOn;
                hasRumbled = false; // Reset rumble when toggling
            }
            lastTrigger = trigger;

            if (shooterOn) {
                shooter.setVelocity(SHOOTER_TICKS_PER_SEC);
            } else {
                shooter.setVelocity(0);
            }

            // ===== SHOOTER ANGLE SERVO =====
            if (gamepad2.x) setServoDegrees(0);
            else if (gamepad2.y) setServoDegrees(45);
            else if (gamepad2.b) setServoDegrees(90);

            // ===== STOPPER =====
            if (gamepad2.dpad_right) stopperS.setPower(1);
            else if (gamepad2.dpad_left) stopperS.setPower(-1);
            else stopperS.setPower(0);

            // ===== DRIVE (Merged) =====
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double max = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            frontLeft.setPower((y + x + rx) / max);
            frontRight.setPower((y - x - rx) / max);
            backLeft.setPower((y - x + rx) / max);
            backRight.setPower((y + x - rx) / max);

            // =================================================================
            //                        FILE 2 AUTO-AIM / VISION LOGIC
            // =================================================================

            /* ================== MANUAL TURRET OVERRIDE ================== */
            double manualTurretPower = 0;
            if (gamepad2.right_bumper) {
                manualTurretPower = -TURRET_POWER;
            } else if (gamepad2.left_bumper) {
                manualTurretPower = TURRET_POWER;
            } else if (Math.abs(gamepad2.right_stick_x) > 0.05) {
                manualTurretPower = -gamepad2.right_stick_x * 0.6;
            }

            // MODIFICATION START: Removed the force disable.
            // If manual input is detected, we don't disable auto-aim permanently.
            // We just override it in the logic below.
            // if (manualTurretPower != 0) {
            //    isAutoAim = false;
            // }
            // MODIFICATION END

            /* ================== AUTO AIM TOGGLE ================== */
            if (gamepad2.square && !lastSquare) {
                isAutoAim = !isAutoAim;
            }
            lastSquare = gamepad2.square;

            /* ================== CALCULATE ANGLES ================== */
            // Robot Heading (World Frame)
            double robotYaw = normalizeAngle(
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset
            );

            // Turret Position relative to Robot
            double turretTicks = turretMotor.getCurrentPosition();
            double turretDeg = turretTicks / TICKS_PER_DEGREE;

            // Turret Heading (World Frame) = Robot Heading + Turret Angle
            double turretAbs = normalizeAngle(robotYaw + turretDeg);

            /* ================== VISION LOGIC ================== */
            boolean tagVisible = false;
            double rawBearing = 0;
            double bearingError = 0;

            // MODIFICATION START: Added check for (manualTurretPower == 0)
            // Auto aim only runs if enabled AND driver is NOT touching the controls.
            if (isAutoAim && manualTurretPower == 0) {
                List<AprilTagDetection> detections = aprilTag.getDetections();

                // Loop through detections to find our specific Tag ID
                for (AprilTagDetection d : detections) {
                    if (d.metadata != null && d.id == RED_GOAL_TAG_ID) {
                        tagVisible = true;

                        // 1. RAW BEARING: The angle from Camera Center to Tag
                        // This uses atan2(x, z) internally.
                        rawBearing = d.ftcPose.bearing;

                        // 2. ERROR CALCULATION
                        bearingError = rawBearing - BEARING_CENTER;

                        // 3. SMOOTHING: Low pass filter to remove camera jitter
                        smoothedBearingError =
                                bearingError * SMOOTHING_ALPHA +
                                        smoothedBearingError * (1.0 - SMOOTHING_ALPHA);

                        // 4. UPDATE TARGET WORLD ANGLE
                        // Since camera is on turret:
                        // Current Turret World Heading + Angle to Tag = New Target Heading
                        targetWorldAngle = normalizeAngle(turretAbs + smoothedBearingError);

                        // 5. SWITCH PID GAINS (Vision is noisy, needs different gains)
                        if (!usingVisionGains) {
                            turretPID = new PIDFController(pidVision);
                            usingVisionGains = true;
                        }
                        break;
                    }
                }

                // If we lost the tag, switch back to Gyro gains for a stiff hold
                if (!tagVisible && usingVisionGains) {
                    turretPID = new PIDFController(pidGyro);
                    usingVisionGains = false;
                }

                // Calculate the angle the turret needs to be at relative to the robot body
                // Target (World) - Robot (World) = Target (Local)
                double errorDeg = normalizeAngle(targetWorldAngle - robotYaw);

                // Safety Clip
                errorDeg = Range.clip(errorDeg, MAX_TURRET_ANGLE_NEGATIVE, MAX_TURRET_ANGLE_POSITIVE);

                turretPID.targetPosition = errorDeg * TICKS_PER_DEGREE;
                double power = turretPID.update(turretTicks);

                // Soft Limits (Prevent cable snapping) - Using File 2 Limits for AutoAim
                if (turretDeg > MAX_TURRET_ANGLE_POSITIVE && power > 0) {
                    power = 0;
                    isAtLimit = true;
                }
                else if (turretDeg < MAX_TURRET_ANGLE_NEGATIVE && power < 0) {
                    power = 0;
                    isAtLimit = true;
                }
                else {
                    isAtLimit = false;
                }

                turretMotor.setPower(Range.clip(power, -1.0, 1.0));
            }
            else {
                // Manual Control (Runs if Manual Power != 0 OR isAutoAim is False)
                // (manualTurretPower was calculated at the start of the vision logic)

                // Soft Limits (File 1 Logic primarily for manual safety)
                int turretPos = turret.getCurrentPosition();
                if (turretPos <= -TURRET_LIMIT_TICKS && manualTurretPower < 0) {
                    manualTurretPower = 0;
                }
                if (turretPos >= TURRET_LIMIT_TICKS && manualTurretPower > 0) {
                    manualTurretPower = 0;
                }

                turretMotor.setPower(manualTurretPower);
                isAtLimit = false;
            }


            // ===== TELEMETRY & RUMBLE (FILE 1) =====
            double currentRPM = (shooter.getVelocity() * 60.0) / HD_HEX_TICKS_PER_REV;

            // Rumble when we hit the target RPM (+/- 50 RPM)
            if (shooterOn && !hasRumbled && currentRPM >= (TARGET_RPM - 50)) {
                gamepad2.rumble(500);
                hasRumbled = true;
            }

            telemetry.addData("Shooter", shooterOn ? "LIMITER ACTIVE" : "OFF");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Current RPM", "%.0f", currentRPM);
            telemetry.addData("Turret Angle", "%.1f°", (turret.getCurrentPosition() / CORE_HEX_TICKS_PER_REV) * 360.0);

            // ===== TELEMETRY (FILE 2) =====
            telemetry.addLine("\n--- TURRET MATH (AUTO) ---");
            telemetry.addData("Mode", isAutoAim ? "AUTO AIM" : "MANUAL");
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addData("Robot Yaw (IMU)", "%.2f", robotYaw);
            telemetry.addData("Turret Pos (Deg Vision)", "%.2f", turretDeg);
            telemetry.addData("Target World Angle", "%.2f", targetWorldAngle);

            telemetry.addLine("\n--- VISION DEBUG ---");
            telemetry.addData("Bearing Error", "%.3f", bearingError);
            telemetry.addData("Using Vision PID", usingVisionGains);

            telemetry.update();
        }

        // Cleanup
        visionPortal.close();
    }

// ---------------- HELPER FUNCTIONS (FILE 1) ----------------

    private void setServoDegrees(double deg) {
        degree.setPosition(Math.min(1.0, Math.max(0.0, deg / 180.0)));
    }

// ---------------- HELPER FUNCTIONS (FILE 2) ----------------

    private double normalizeAngle(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private void setManualExposure(long exposureMS, int gain) {
        if(visionPortal == null) return;
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (exposure.getMode() != ExposureControl.Mode.Manual) {
            exposure.setMode(ExposureControl.Mode.Manual);
        }
        exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        gainControl.setGain(gain);
    }

    private void initVision() {
        // Adjust these numbers to match where your camera is on the turret!
        Position camPos = new Position(DistanceUnit.CM, 0, 6, 43, 0);
        YawPitchRollAngles camRot = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(camPos, camRot)
                .setDrawTagID(true)
                .setDrawCubeProjection(false) // Saves CPU
                .build();

        aprilTag.setDecimation(DECIMATION_SEARCH);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(streamProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Usually faster
                .build();

        FtcDashboard.getInstance().startCameraStream(streamProcessor, 0);
    }

    // Dashboard Stream Processor (From File 2)
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1,1,Bitmap.Config.RGB_565));

        public void init(int w, int h, CameraCalibration c) {
            lastFrame.set(Bitmap.createBitmap(w,h,Bitmap.Config.RGB_565));
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