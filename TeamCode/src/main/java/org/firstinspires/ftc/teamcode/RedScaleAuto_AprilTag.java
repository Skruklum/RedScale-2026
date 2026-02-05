package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controllers.RobotPoseController;
import org.firstinspires.ftc.teamcode.controllers.ShooterRotatorController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Autonomous(name = "Red_Scale_Test_Auto_AprilTag", group = "Autonomous Test")
public class RedScaleAuto_AprilTag extends LinearOpMode {

    private ElapsedTime opModeTime = new ElapsedTime();
    private RobotPoseController robotPoseController;
    private ShooterRotatorController shooterRotatorController;
    private VisionCamera visionCamera;

    private MecanumDrive drive;


    // Set this to true for Red Alliance, false for Blue
    private boolean isBlue = false;

    public Pose2d reflect(double x, double y, double degrees) {
        return isBlue ? new Pose2d(x, -y, -degrees) : new Pose2d(x, y, degrees);
    }

    public Vector2d reflectV(double x, double y) {
        return isBlue ? new Vector2d(x, -y) : new Vector2d(x, y);
    }

    public double reflect(double value) {
        return isBlue ? -value : value;
    }


    @Override
    public void runOpMode() {

        robotPoseController = new RobotPoseController(hardwareMap);
        shooterRotatorController = new ShooterRotatorController(hardwareMap, robotPoseController, "shooterRot");
        visionCamera = new VisionCamera(hardwareMap);

        // Initialize your MecanumDrive (this contains your 2-dead wheel localizer)
        // Make sure the starting Pose matches your MeepMeep code exactly
        Pose2d initialPose = reflect(-60, 35, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);

        while (true) {
            if (gamepad1.crossWasPressed()) {
                isBlue = !isBlue;

                if (isBlue) visionCamera.setBlueAlliance();
                else if (!isBlue) visionCamera.setRedAlliance();
            }

            telemetry.addData("CURRENT TEAM (press cross on gamepad 1 to change):", isBlue ? "BLUE" : "RED");
            telemetry.update();

            if (isStarted()) {
                break;
            }
            if (isStopRequested()) {
                break;
            }
        }

        waitForStart();

        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(initialPose)
                .afterTime(0, () -> {shooterRotatorController.setTargetWorldAngle(reflect(55));})
//                .stopAndAdd(shooter.setState(true))
                .stopAndAdd(() -> {
                    AprilTagDetection detectedAprilTag = visionCamera.getAprilTag();
                    double robotPoleX = detectedAprilTag.robotPose.getPosition().x;
                    double robotPoleY = detectedAprilTag.robotPose.getPosition().y;
                    double robotPoleZ = detectedAprilTag.robotPose.getPosition().z;

                    double detectedYaw = detectedAprilTag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    drive.localizer.setPose(new Pose2d(robotPoleX, robotPoleY, Math.toRadians(detectedYaw)));


                })
                .strafeTo(reflectV(-27.5, 30))
//                .afterTime(0, intake.setPower(1))

//                .stopAndAdd(stopper.timedPower(1.0))
                .waitSeconds(2)
//                .stopAndAdd(stopper.timedPower(-1.0))

                .splineToConstantHeading(reflectV(-1.5, 25), reflect(Math.toRadians(90.00)))
                .splineToConstantHeading(reflectV(-1.5, 62.5), reflect(Math.toRadians(90.00)))
                .strafeTo(reflectV(-27.5, 36.5))

//                .stopAndAdd(stopper.timedPower(1.0))
                .waitSeconds(2)
//                .stopAndAdd(stopper.timedPower(-1.0))

                .splineToConstantHeading(reflectV(23, 25), reflect(Math.toRadians(90.00)))
                .splineToConstantHeading(reflectV(23, 68.5), reflect(Math.toRadians(90.00)))
                .strafeTo(reflectV(26, 60))
                .strafeTo(reflectV(-26, 39.5))

//                .stopAndAdd(stopper.timedPower(1.0))
                .afterTime(0, intake.setPower(1))
                .waitSeconds(2);
//                .stopAndAdd(stopper.timedPower(-1.0));

        if (isStopRequested()) return;

        // Build and execute the action
        Action driveAction = trajectoryActionBuilder
                .build();

        while (opModeIsActive()) {
            opModeTime.reset();
            runBlocking(driveAction);
        }
    }


    public void runBlocking(Action action) {
//        FtcDashboard dash = FtcDashboard.getInstance();
//        Canvas previewCanvas = new Canvas();
//        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
            packet.put("time", opModeTime);

            robotPoseController.update();
            shooterRotatorController.update();

            shooterRotatorController.activate();

            running = action.run(packet);

            Pose2d pose = drive.localizer.getPose();

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


//            extendo.runAuto();
//            lifter.runAuto();
////            lifter.sendTelemetryAuto(packet);
//            dash.sendTelemetryPacket(packet);
        }
    }

}