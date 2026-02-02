package org.firstinspires.ftc.teamcode.testers;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
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
@TeleOp(name ="AprilTag AutoAim CENTER + LIMITER", group = "Test")
public class AprilTagTracking2Agi extends OpMode {

    /* ================== PID ================== */
    public static PIDCoefficients pidVision = new PIDCoefficients(0.015, 0, 0.7);
    public static PIDCoefficients pidGyro   = new PIDCoefficients(0.035, 0, 0.001);

    /* ================== TURRET ================== */
    private static final double MAX_TURRET_ANGLE_POSITIVE = 170;
    private static final double MAX_TURRET_ANGLE_NEGATIVE = -150;
    private static final double TICKS_PER_DEGREE = 2.838;

    /* ================== VISION ================== */
    public static double BEARING_CENTER = 1.5;
    private static final double SMOOTHING_ALPHA = 0.2;

    /* ================== CAMERA ================== */
    private static final long EXPOSURE_MS = 6;
    private static final int CAMERA_GAIN = 250;
    private static final float DECIMATION_SEARCH = 2.0f;

    public static int RED_GOAL_TAG_ID = 20;

    enum AimState {
        SNAP_TO_BEARING,
        LOCK_WORLD
    }

    private AimState aimState = AimState.SNAP_TO_BEARING;

    /* ================== HARDWARE ================== */
    private DcMotorEx turretMotor;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    /* ================== VISION ================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final CameraStreamProcessor streamProcessor = new CameraStreamProcessor();

    /* ================== STATE ================== */
    private PIDFController turretPID = new PIDFController(pidGyro);
    private boolean usingVisionGains = false;

    private boolean isAutoAim = false;
    private boolean lastSquare = false;
    private boolean isAtLimit = false;

    private double yawOffset = 0;
    private double targetWorldAngle = 0;
    private double smoothedBearingError = 0;

    /* ================== INIT ================== */
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        turretMotor = hardwareMap.get(DcMotorEx.class, "shooterRot");
        frontLeft  = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft   = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight  = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);

        turretPID.setOutputBounds(-1.0, 1.0);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();

        initVision();
    }

    @Override
    public void start() {
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        setManualExposure(EXPOSURE_MS, CAMERA_GAIN);
    }

    /* ================== LOOP ================== */
    @Override
    public void loop() {

        /* ===== DRIVE ===== */
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double max = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
        frontLeft.setPower((y + x + rx) / max);
        frontRight.setPower((y - x - rx) / max);
        backLeft.setPower((y - x + rx) / max);
        backRight.setPower((y + x - rx) / max);

        /* ===== AUTO AIM TOGGLE ===== */
        if (gamepad2.square && !lastSquare) {
            isAutoAim = !isAutoAim;
            aimState = AimState.SNAP_TO_BEARING;
        }
        lastSquare = gamepad2.square;

        double robotYaw = normalizeAngle(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset
        );

        double turretTicks = turretMotor.getCurrentPosition();
        double turretDeg = turretTicks / TICKS_PER_DEGREE;
        double turretAbs = normalizeAngle(robotYaw + turretDeg);

        boolean tagVisible = false;
        double rawBearing = 0;
        double bearingError = 0;

        if (isAutoAim) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection d : detections) {
                if (d.metadata != null && d.id == RED_GOAL_TAG_ID) {
                    tagVisible = true;

                    rawBearing = d.ftcPose.bearing;
                    bearingError = rawBearing - BEARING_CENTER;

                    smoothedBearingError =
                            bearingError * SMOOTHING_ALPHA +
                                    smoothedBearingError * (1.0 - SMOOTHING_ALPHA);

                    if (aimState == AimState.SNAP_TO_BEARING) {
                        targetWorldAngle =
                                normalizeAngle(turretAbs + smoothedBearingError);

                        if (Math.abs(smoothedBearingError) < 0.5) {
                            aimState = AimState.LOCK_WORLD;
                        }
                    }

                    if (!usingVisionGains) {
                        turretPID = new PIDFController(pidVision);
                        usingVisionGains = true;
                    }
                    break;
                }
            }

            double errorDeg = normalizeAngle(targetWorldAngle - robotYaw);
            errorDeg = Range.clip(
                    errorDeg,
                    MAX_TURRET_ANGLE_NEGATIVE,
                    MAX_TURRET_ANGLE_POSITIVE
            );

            turretPID.targetPosition = errorDeg * TICKS_PER_DEGREE;
            double power = turretPID.update(turretTicks);

            /* ===== LIMITER ===== */
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
            double manual = -gamepad2.right_stick_x * 0.6;

            if (turretDeg > MAX_TURRET_ANGLE_POSITIVE && manual > 0) manual = 0;
            if (turretDeg < MAX_TURRET_ANGLE_NEGATIVE && manual < 0) manual = 0;

            turretMotor.setPower(manual);
            isAtLimit = false;
        }

        /* ================== TELEMETRY ================== */
        telemetry.addLine("===== AUTO AIM DEBUG =====");
        telemetry.addData("AutoAim", isAutoAim);
        telemetry.addData("AimState", aimState);
        telemetry.addData("AtLimit", isAtLimit);

        telemetry.addLine("----- ANGLES -----");
        telemetry.addData("RobotYaw", robotYaw);
        telemetry.addData("TurretDeg", turretDeg);
        telemetry.addData("TurretAbs", turretAbs);
        telemetry.addData("TargetWorld", targetWorldAngle);

        telemetry.addLine("----- VISION -----");
        telemetry.addData("TagVisible", tagVisible);
        telemetry.addData("RawBearing", rawBearing);
        telemetry.addData("BearingCenter", BEARING_CENTER);
        telemetry.addData("BearingError", bearingError);
        telemetry.addData("SmoothedError", smoothedBearingError);

        telemetry.addLine("----- PID -----");
        telemetry.addData("PID Mode", usingVisionGains ? "VISION" : "GYRO");
        telemetry.addData("PID Target (ticks)", turretPID.targetPosition);
        telemetry.addData("TurretTicks", turretTicks);

        telemetry.update();
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

    /* ================== UTILS ================== */
    private double normalizeAngle(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private void setManualExposure(long exposureMS, int gain) {
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        gainControl.setGain(gain);
    }

    private void initVision() {
        Position camPos = new Position(DistanceUnit.CM, 0, 6, 43, 0);
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
