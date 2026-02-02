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
import org.firstinspires.ftc.teamcode.controllers.ShooterController;
import org.firstinspires.ftc.teamcode.controllers.ShooterSolution;
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
@TeleOp(name ="April Tag Shooting Test (DYNAMIC)", group = "Test")
public class aprilTagShooting extends OpMode {
    private double SHOOTER_MOTOR_COUNTS_PER_REV = 28.0;


    public static PIDCoefficients pidGyro = new PIDCoefficients(0.035, 0, 0.001);

    PIDFController turretPIDController = new PIDFController(pidGyro);
    private boolean usingVisionGains = false;


    // Shooter Settings

    // Turret Limits
    // Mechanical Limits (Degrees relative to robot front)
    private static final double MAX_TURRET_ANGLE_POSITIVE = 170; // Left Limit
    private static final double MAX_TURRET_ANGLE_NEGATIVE = -150; // Right Limit
    private static final double TICKS_PER_DEGREE = 2.838;

    // Alliance & Tags
    public static int RED_GOAL_TAG_ID = 20;
    public static int BLUE_GOAL_TAG_ID = 24;
    private int activeGoalTagId = RED_GOAL_TAG_ID;
    private boolean isRedAlliance = true;

    // Hardware
    private DcMotorEx turretMotor, shooterMotor;
    private Servo shooterAd;

    private IMU imu;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final CameraStreamProcessor streamProcessor = new CameraStreamProcessor();

    // State
    private double targetWorldAngle = 0;
    private boolean isAutoAim = false;
    private boolean isAtLimit = false;
    private double yawOffset = 0;

    private double detectedDistance = 0;

    // Input Toggles
    private boolean gamepad2_isSquareClicked = false;


    // Driver 1 Toggles
    private boolean lastDpadDown = false;

    // Camera Settings
    private static final long EXPOSURE_MS = 6;
    private static final int CAMERA_GAIN = 250;
    private static final float DECIMATION_SEARCH = 2.0f;
    private static final double SMOOTHING_ALPHA = 0.2;
    private double smoothedBearing = 0;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private double AprilTagDistance = 0;

    private double AprilTagDataA = 0;

    FtcDashboard dashboard;

    public static PIDFCoefficients coeffs = new PIDFCoefficients(60,0,0,17.5);

    private ShooterController shooterController;


    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Motors
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterAd = hardwareMap.get(Servo.class, "shooterAd");

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

        // Turret Config
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // INCREASED LIMIT TO 1.0 (Full Speed)
        turretPIDController.setOutputBounds(-1.0, 1.0);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

        shooterController = new ShooterController();

        // IMU Config
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)
        );
        imu.initialize(parameters);
        imu.resetYaw();

        initVision();
        telemetry.addLine("Ready. Gain Scheduling Enabled.");
        telemetry.update();
    }

    @Override
    public void start() {
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        setManualExposure(EXPOSURE_MS, CAMERA_GAIN);
    }

    @Override
    public void loop() {

        // ===== DRIVE =====
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double max = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
        frontLeft.setPower((y + x + rx) / max);
        frontRight.setPower((y - x - rx) / max);
        backLeft.setPower((y - x + rx) / max);
        backRight.setPower((y + x - rx) / max);

        // --- Alliance Selection (Driver 1) ---
        boolean currentDpadDown = gamepad1.dpad_down;
        if (currentDpadDown && !lastDpadDown) {
            isRedAlliance = !isRedAlliance;
            activeGoalTagId = isRedAlliance ? RED_GOAL_TAG_ID : BLUE_GOAL_TAG_ID;
        }
        lastDpadDown = currentDpadDown;


        // --- Turret / Auto Aim ---
        if (gamepad2.square && !gamepad2_isSquareClicked) {
            isAutoAim = !isAutoAim;
            if(isAutoAim) {
                // Lock heading on enable
                double robotYaw = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset);
                double turretDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
                targetWorldAngle = normalizeAngle(robotYaw + turretDeg);
            }
            gamepad2_isSquareClicked = true;
        } else if (!gamepad2.square) {
            gamepad2_isSquareClicked = false;
        }

        double robotYaw = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset);
        double turretTicks = turretMotor.getCurrentPosition();
        double turretDegrees = turretTicks / TICKS_PER_DEGREE;
        boolean tagVisible = false;

        double turretDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
        double absoluteDegreeRelativeToRobot = normalizeAngle(robotYaw + turretDeg);

        if (isAutoAim) {
            // Search Detections
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection d : detections) {

                telemetry.addData("detectedId", d.id);
                telemetry.addData("d.metadata.id", d.metadata.id);

                telemetry.addData("d.ftcPose.range", d.ftcPose.range);
                telemetry.addData("d.ftcPose.bearing", d.ftcPose.bearing);


                if (d.id == activeGoalTagId) {
                    if (d.ftcPose.range != 0) {
                        AprilTagDistance = d.ftcPose.range;
                    }

                    if (d.ftcPose.bearing != 0) {
                        AprilTagDataA = d.ftcPose.bearing;

                    }
                }


                if (d.metadata != null && d.id == activeGoalTagId) {
                    telemetry.addLine("DETECTED!!!");

                    detectedDistance = d.ftcPose.range;

                    tagVisible = true;

                    break;
                }
            }


        } else {
            // Manual Mode
            double manualInput = -gamepad2.right_stick_x;
            double speedMod = Range.clip(0.6 + (-gamepad2.right_stick_y * 0.4), 0.2, 1.0);
            double manualPower = manualInput * speedMod;

            if (turretDegrees > MAX_TURRET_ANGLE_POSITIVE && manualPower > 0) manualPower = 0;
            if (turretDegrees < MAX_TURRET_ANGLE_NEGATIVE && manualPower < 0) manualPower = 0;

            turretMotor.setPower(manualPower);
            isAtLimit = false;
        }


        if (tagVisible) {
            ShooterSolution shooterSolution = shooterController.getBestShootingSolution(inchToCm(AprilTagDistance), 98.5);
            shooterAd.setPosition(shooterController.angleToServo(shooterSolution.bestAngle));
            double targetVelocityTicks = shooterController.rpmToVelocityTicks(shooterSolution.targetRPM);
            shooterMotor.setVelocity(targetVelocityTicks);


            double SHOOTER_RPM = shooterMotor.getVelocity() / SHOOTER_MOTOR_COUNTS_PER_REV * 60;

            telemetry.addData("SHOOTER VELOCITY", shooterMotor.getVelocity());
            telemetry.addData("SHOOTER RPM", SHOOTER_RPM);


        }

        // Telemetry Updates
        telemetry.addData("ALLIANCE", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("TARGET TAG", activeGoalTagId);
        telemetry.addData("AprilTagDistance", AprilTagDistance);

        telemetry.addData("absoluteDegreeRelativeToRobot (degree)", absoluteDegreeRelativeToRobot);

        telemetry.addData("AprilTagDataA", AprilTagDataA);
        telemetry.addData("MODE", isAutoAim ? (tagVisible ? "AUTO (VISION)" : "AUTO (GYRO)") : "MANUAL");
        telemetry.addData("PID", isAutoAim ? (usingVisionGains ? "Vision" : "Gyro") : "Manual");
        telemetry.update();
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private void setManualExposure(long exposureMS, int gain) {
        if (visionPortal == null) return;
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    private void initVision() {
        Position cameraPosition = new Position(DistanceUnit.CM, cmToInch(-21.5), mmToInch(39.11),cmToInch(41), 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,0, 0, 0,0);
        aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).setDrawTagOutline(true).setDrawTagID(true).build();
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
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
        public void init(int width, int height, CameraCalibration calibration) { lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565)); }
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = lastFrame.get();
            if ((b.getWidth() != frame.width()) || (b.getHeight() != frame.height())) {
                b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
                lastFrame.set(b);
            }
            Utils.matToBitmap(frame, b);
            return null;
        }
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) { continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get())); }
    }

    /**
     * Converts a measurement from inches to centimeters.
     * @param inch The value in inches.
     * @return The value converted to centimeters.
     */
    public double inchToCm(double inch) {
        return inch * 2.54;
    }

    /**
     * Converts a measurement from inches to centimeters.
     * @param cm The value in inches.
     * @return The value converted to centimeters.
     */
    public double cmToInch(double cm) {
        return cm / 2.54;
    }

    /**
     * Converts a measurement from inches to centimeters.
     * @param mm The value in inches.
     * @return The value converted to centimeters.
     */
    public double mmToInch(double mm) {
        double cm = mm / 10;
        return cm / 2.54;
    }

}