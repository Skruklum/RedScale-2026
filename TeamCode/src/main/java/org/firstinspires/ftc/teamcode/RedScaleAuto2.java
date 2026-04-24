package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.RobotPoseController;
import org.firstinspires.ftc.teamcode.controllers.ShooterRotatorController;

@Autonomous(name = "Red_Scale_Auto_PLAYOFF", group = "Autonomous")
public class RedScaleAuto2 extends LinearOpMode {

    private ElapsedTime opModeTime = new ElapsedTime();
    private RobotPoseController robotPoseController;
    private ShooterRotatorController shooterRotatorController;

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

    private Intake intake;
    private Shooter shooter;
    private Stopper stopper;

    @Override
    public void runOpMode() {

        robotPoseController = new RobotPoseController(hardwareMap);
        shooterRotatorController = new ShooterRotatorController(hardwareMap, robotPoseController, "shooterRot");

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        stopper = new Stopper(hardwareMap);

        robotPoseController.resetYaw();

        while (true) {
            if (gamepad1.crossWasPressed()) {
                isBlue = !isBlue;
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

        // Initialize your MecanumDrive (this contains your 2-dead wheel localizer)
        // Make sure the starting Pose matches your MeepMeep code exactly
        Pose2d initialPose = reflect(62, 10, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectoryActionBuilder = drive.actionBuilder(initialPose)
                        .strafeTo(reflectV(62, 50))
                                .waitSeconds(3);
        waitForStart();

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

            running = action.run(packet);

            Pose2d pose = drive.localizer.getPose();

            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("Shooter Current Velocity", shooter.getVelocity());
            telemetry.update();

//            extendo.runAuto();
//            lifter.runAuto();
////            lifter.sendTelemetryAuto(packet);
//            dash.sendTelemetryPacket(packet);
        }
    }

}