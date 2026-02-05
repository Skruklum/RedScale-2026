package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
import org.firstinspires.ftc.teamcode.testers.aprilTagTrackingAim;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class VisionCamera {
    /* ================== VISION OBJECTS ================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final aprilTagTrackingAim.CameraStreamProcessor streamProcessor = new aprilTagTrackingAim.CameraStreamProcessor();

    /* ================== CAMERA SETTINGS ================== */
    private static final long EXPOSURE_MS = 6;
    private static final int CAMERA_GAIN = 250;
    private static final float DECIMATION_SEARCH = 2.0f;

    private boolean isBlue = false;

    public static int RED_GOAL_TAG_ID = 24;
    public static int BLUE_GOAL_TAG_ID = 20;
    private int activeGoalTagId = RED_GOAL_TAG_ID;

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

    public void setBlueAlliance() {
        isBlue  = true;
        activeGoalTagId = BLUE_GOAL_TAG_ID;
    }

    public void setRedAlliance() {
        isBlue  = false;
        activeGoalTagId = RED_GOAL_TAG_ID;
    }


    public VisionCamera(HardwareMap hardwareMap) {
        // Adjust these numbers to match where your camera is on the turret!
        Position cameraPosition = new Position(DistanceUnit.INCH, cmToInch(-21.5), mmToInch(39.11),cmToInch(41), 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,-90, 0, 0,0);

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

    public AprilTagDetection getAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection DetectedTag = null;

        for (AprilTagDetection d : detections) {
            if (d.metadata != null && d.id == activeGoalTagId) {
                DetectedTag = d;
                break;
            }
        }

        return DetectedTag;
    }

    public AprilTagDetection getAprilTag(double targetId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection DetectedTag = null;

        for (AprilTagDetection d : detections) {
            if (d.metadata != null && d.id == targetId) {
                DetectedTag = d;
                break;
            }
        }

        return DetectedTag;
    }

    public List<AprilTagDetection> getAprilTags() {
        return aprilTag.getDetections();
    }

    public void stop() {
        visionPortal.stopStreaming();
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
