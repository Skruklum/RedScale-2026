package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class RsudAngleFixIniYangBenerCoHold extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotorEx shooter;
    private DcMotor intake;
    private DcMotorEx turret;
    private Servo degree;
    private CRServo stopperS;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // ---------------- VISION ----------------
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ---------------- CONSTANTS ----------------
    static final double BARE_MOTOR_TICKS_PER_REV = 28.0;
    static final double TURRET_GEAR_RATIO = 4.75;
    static final double CORE_HEX_TICKS_PER_REV = 288.0 * TURRET_GEAR_RATIO;
    static final double TURRET_KP = 0.02;
    static final double TURRET_MAX_POWER = 0.6;
    static final double TURRET_LIMIT_DEG = 60.0;
    static final int TURRET_LIMIT_TICKS =
            (int) ((TURRET_LIMIT_DEG / 360.0) * CORE_HEX_TICKS_PER_REV);

    // ---------------- STATE ----------------
    boolean shooterOn = false;
    boolean lastTrigger = false;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        turret      = hardwareMap.get(DcMotorEx.class, "shooterRot");
        degree      = hardwareMap.get(Servo.class, "shooterAd");
        intake      = hardwareMap.get(DcMotor.class, "intake");
        shooter     = hardwareMap.get(DcMotorEx.class, "shooter");
        stopperS    = hardwareMap.get(CRServo.class, "stopper");

        frontLeft   = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight  = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft    = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight   = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )));

        // ---------------- VISION INIT ----------------
        initAprilTag();
        setManualExposure(6, 250);

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

        telemetry.addLine("Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== INTAKE (NORMAL HOLD) =====
            if (gamepad1.right_bumper) {
                intake.setPower(1);      // FORWARD
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);     // REVERSE
            } else {
                intake.setPower(0);      // STOP
            }

            // ===== SHOOTER TOGGLE =====
            boolean trigger = gamepad2.right_trigger > 0.2;
            if (trigger && !lastTrigger) shooterOn = !shooterOn;
            lastTrigger = trigger;
            shooter.setPower(shooterOn ? 1.0 : 0.0);

            // ===== TURRET MANUAL / AUTO =====
            boolean manualOverride = false;
            if (gamepad2.right_bumper) {
                turret.setPower(-1);
                manualOverride = true;
            } else if (gamepad2.left_bumper) {
                turret.setPower(1);
                manualOverride = true;
            }

            if (!manualOverride) {
                updateCameraLogic();
            }

            // ===== SHOOTER ANGLE SERVO =====
            if (gamepad2.x) setServoDegrees(0);
            else if (gamepad2.y) setServoDegrees(45);
            else if (gamepad2.b) setServoDegrees(90);

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

            // ===== TELEMETRY =====
            int turretTicks = turret.getCurrentPosition();
            double turretDeg = (turretTicks / CORE_HEX_TICKS_PER_REV) * 360.0;
            double shooterRPM = (shooter.getVelocity() * 60.0) / BARE_MOTOR_TICKS_PER_REV;

            List<AprilTagDetection> tags = aprilTag.getDetections();

            telemetry.addData("Turret Ticks", turretTicks);
            telemetry.addData("Turret Angle", "%.1f°", turretDeg);
            telemetry.addData("Tag Found", tags.size() > 0 ? "YES" : "NO");

            if (!tags.isEmpty() && tags.get(0).ftcPose != null) {
                telemetry.addData("Target X", "%.3f", tags.get(0).ftcPose.x);
            }

            telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
            telemetry.addData("RPM", "%.0f", shooterRPM);
            telemetry.update();
        }

        visionPortal.close();
    }

    // ---------------- FUNCTIONS ----------------

    private void setServoDegrees(double deg) {
        degree.setPosition(Math.min(1.0, Math.max(0.0, deg / 180.0)));
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        while (!isStopRequested() &&
                visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        ExposureControl exposureControl =
                visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl =
                visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    private void updateCameraLogic() {
        int turretPos = turret.getCurrentPosition();
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) {
            turret.setPower(0);
            return;
        }

        AprilTagDetection tag = detections.get(0);
        if (tag.ftcPose == null) {
            turret.setPower(0);
            return;
        }

        double xError = tag.ftcPose.x;
        double power = clamp(-xError * TURRET_KP);

        if (turretPos <= -TURRET_LIMIT_TICKS && power < 0) power = 0;
        if (turretPos >=  TURRET_LIMIT_TICKS && power > 0) power = 0;

        turret.setPower(power);
    }

    private double clamp(double value) {
        return Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, value));
    }
}