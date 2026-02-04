package org.firstinspires.ftc.teamcode.testers;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
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
@TeleOp(name ="AprilTag Strafe Idk", group = "Test")
public class TrackStrafeIdkManShit extends OpMode {

    /* ================== RATIO ================== */
    private double coordinatePerInchRatio = 1;

    /* ================== PID ================== */
    // PID for when we see the tag (needs to be responsive)
    public static PIDCoefficients pidVision = new PIDCoefficients(0.01, 0, 0.001);
    // PID for holding position when tag is lost (needs to be stiff)
    public static PIDCoefficients pidGyro   = new PIDCoefficients(0.035, 0, 0.002);

    /* ================== TURRET SETTINGS ================== */
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

    public static int RED_GOAL_TAG_ID = 20;

    /* ================== HARDWARE ================== */
    private DcMotorEx turretMotor;
    private IMU imu;

    /* ================== VISION OBJECTS ================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final CameraStreamProcessor streamProcessor = new CameraStreamProcessor();

    /* ================== APRIL TAG DETECTED DATA ================== */
    private double detectedDistanceInch = 0;
    private double detectedDistanceX_Inch = 0;
    private double detectedDistanceY_Inch = 0;

    private double aprilTagX = 0;
    private double aprilTagY = 0;

    private double robotPoseX = 0;
    private double robotPoseY = 0;
    private double robotPoseYaw = 0;

    private Pose2d RedAlliance_AprilTag_Position = new Pose2d(62.51,56.74,Math.toRadians(-40));
    private Pose2d BlueAlliance_AprilTag_Position = new Pose2d(62.51,-56.74,Math.toRadians(40));

    enum AimState {
        SNAP_TO_BEARING,
        LOCK_WORLD
    }

    private AimState aimState = AimState.SNAP_TO_BEARING;


    /* ================== STATE VARIABLES ================== */
    private PIDFController turretPID = new PIDFController(pidGyro);
    private boolean usingVisionGains = false;

    private boolean isAutoAim = false;
    private boolean lastSquare = false;
    private boolean isAtLimit = false;
    private boolean fieldOrientedLock = false;


    private double yawOffset = 0;
    private double targetWorldAngle = 0; // The "World Heading" we want to face
    private double smoothedBearingError = 0;

    private double absAngleTargetAprilTag = 0;

    private MecanumDrive mecanumDrive;


    @Override
    public void init() {
        // Setup Telemetry to Dashboard and Driver Station
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Mapping
        turretMotor = hardwareMap.get(DcMotorEx.class, "shooterRot");

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        mecanumDrive = new MecanumDrive(hardwareMap, initialPose);

        // Turret Configuration
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);

        turretPID.setOutputBounds(-1.0, 1.0);

        // IMU Init
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
        // Set the zero-yaw offset
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        setManualExposure(EXPOSURE_MS, CAMERA_GAIN);
    }

    @Override
    public void loop() {
        /* ================== DRIVER CONTROL ================== */
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

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

        if (gamepad2.circleWasPressed()) {
            targetWorldAngle = normalizeAngle(robotYaw + turretDeg);
        }

        /* ================== VISION LOGIC ================== */
        boolean tagVisible = false;
        double rawBearing = 0;
        double bearingError = 0;

        if (gamepad1.triangleWasPressed()) {
            fieldOrientedLock = !fieldOrientedLock;
        }

        if (isAutoAim) {
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

                    if (aimState == AimState.SNAP_TO_BEARING) {
                        targetWorldAngle =
                                normalizeAngle(turretAbs + smoothedBearingError);

                         robotPoseX = d.robotPose.getPosition().x;
                         robotPoseY = d.robotPose.getPosition().y;
                         robotPoseYaw = d.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                         detectedDistanceInch = d.ftcPose.range;
                         detectedDistanceX_Inch = Math.sin(rawBearing) * detectedDistanceInch;
                         detectedDistanceY_Inch = Math.cos(rawBearing) * detectedDistanceInch;

                        aprilTagX = d.metadata.fieldPosition.get(0);
                        aprilTagY = d.metadata.fieldPosition.get(1);

                        mecanumDrive.localizer.setPose(new Pose2d(robotPoseX, robotPoseY, Math.toRadians(robotPoseYaw)));


                        if (Math.abs(smoothedBearingError) < 0.5) {
                            aimState = AimState.LOCK_WORLD;
                        }
                    }

                    // 5. SWITCH PID GAINS (Vision is noisy, needs different gains)
                    if (!usingVisionGains) {
                        turretPID = new PIDFController(pidVision);
                        usingVisionGains = true;
                    }
                    break;
                }
            }

            // If we lost the tag, switch back to Gyro gains for a stiff hold
            if (fieldOrientedLock && aprilTagX != 0 && aprilTagY != 0) {
                if (Math.abs(smoothedBearingError) > 0.5) aimState = AimState.SNAP_TO_BEARING;

                turretPID = new PIDFController(pidGyro);

                double robotPoseX = mecanumDrive.localizer.getPose().position.x;
                double robotPoseY = mecanumDrive.localizer.getPose().position.y;

                double deltaX = (robotPoseX - aprilTagX);
                double deltaY = (robotPoseY - aprilTagY);

                // Calculate distance for telemetry
                double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

                // Calculate the world angle needed to point at the target
                // atan2 gives us the correct angle in all quadrants
                double angleToTarget = Math.toDegrees(Math.atan2(deltaX, deltaY));

                telemetry.addLine("\n--- FIELD ORIENTED LOCK ---");
                telemetry.addData("Field Oriented aprilTagX", aprilTagX);
                telemetry.addData("Field Oriented aprilTagY", aprilTagY);
                telemetry.addData("Robot X", robotPoseX);
                telemetry.addData("Robot Y", robotPoseY);
                telemetry.addData("Delta X", deltaX);
                telemetry.addData("Delta Y", deltaY);
                telemetry.addData("Distance to Target", distanceToTarget);
                telemetry.addData("Calculated Angle to Target", angleToTarget);

                Pose2d currentPose = mecanumDrive.localizer.getPose();
                double dx = aprilTagX - currentPose.position.x;
                double dy = aprilTagY - currentPose.position.y;

                // 1. Calculate the absolute world heading needed to face the tag
                 absAngleTargetAprilTag = Math.toDegrees(Math.atan2(dy, dx));
                telemetry.addData("absAngleTargetAprilTag", absAngleTargetAprilTag);


            } else if (!tagVisible && usingVisionGains) {
                if (smoothedBearingError > 0.5) aimState = AimState.SNAP_TO_BEARING;

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

            // Soft Limits (Prevent cable snapping)
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
            // Manual Control
            double manual = -gamepad2.right_stick_x * 0.6;

            // Manual Limits
            if (turretDeg > MAX_TURRET_ANGLE_POSITIVE && manual > 0) manual = 0;
            if (turretDeg < MAX_TURRET_ANGLE_NEGATIVE && manual < 0) manual = 0;

            turretMotor.setPower(manual);
            isAtLimit = false;
        }

        /* ================== DEBUGGING TELEMETRY ================== */
        // Status
        telemetry.addData("Mode", isAutoAim ? "AUTO AIM" : "MANUAL");
        telemetry.addData("fieldOrientedLock", fieldOrientedLock);

        telemetry.addData("Tag Visible", tagVisible);
        telemetry.addData("AimState", aimState);


        telemetry.addLine("\n--- APRIL TAG ---");
        telemetry.addData("aprilTagX", aprilTagX);
        telemetry.addData("aprilTagY", aprilTagY);

        telemetry.addData("robotPoseX", robotPoseX);
        telemetry.addData("robotPoseY", robotPoseY);
        telemetry.addData("robotPoseYaw", robotPoseYaw);

        telemetry.addData("detectedDistanceInch", detectedDistanceInch);
        telemetry.addData("detectedDistanceX_Inch", detectedDistanceX_Inch);
        telemetry.addData("detectedDistanceY_Inch", detectedDistanceY_Inch);

        telemetry.addLine("\n--- TURRET MATH ---");
        telemetry.addData("Robot Yaw (IMU)", "%.2f", robotYaw);
        telemetry.addData("Turret Pos (Deg)", "%.2f", turretDeg);
        telemetry.addData("Turret Abs (World)", "%.2f", turretAbs);
        telemetry.addData("Target World Angle", "%.2f", targetWorldAngle);

        telemetry.addLine("\n--- VISION DEBUG ---");
        telemetry.addData("Raw Bearing", "%.3f", rawBearing);
        telemetry.addData("Bearing Error", "%.3f", bearingError);
        telemetry.addData("Smoothed Error", "%.3f", smoothedBearingError);
        telemetry.addData("PID Error (Deg)", "%.3f", (turretPID.targetPosition - turretTicks) / TICKS_PER_DEGREE);

        telemetry.addLine("\n--- HARDWARE ---");
        telemetry.addData("Motor Power", "%.2f", turretMotor.getPower());
        telemetry.addData("Limit Reached", isAtLimit);
        telemetry.addData("Using Vision PID", usingVisionGains);

        mecanumDrive.updatePoseEstimate();

        Pose2d pose = mecanumDrive.localizer.getPose();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.update();
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

    /* ================== HELPERS ================== */
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
        Position cameraPosition = new Position(DistanceUnit.CM, cmToInch(-21.5), mmToInch(39.11),cmToInch(41), 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,0, 0, 0,0);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
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

    // Dashboard Stream Processor
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