package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.RobotPoseController;
import org.firstinspires.ftc.teamcode.controllers.ShooterRotatorController;

@Autonomous(name = "Red_Scale_Test_Auto_Red", group = "Autonomous")
public class RedScaleAuto extends LinearOpMode {

    private ElapsedTime opModeTime = new ElapsedTime();
    private RobotPoseController robotPoseController;
    private ShooterRotatorController shooterRotatorController;

    private MecanumDrive drive;
    @Override
    public void runOpMode() {

        robotPoseController = new RobotPoseController(hardwareMap);
        shooterRotatorController = new ShooterRotatorController(hardwareMap, robotPoseController, "shooterRot");


        // Initialize your MecanumDrive (this contains your 2-dead wheel localizer)
        // Make sure the starting Pose matches your MeepMeep code exactly
        Pose2d initialPose = new Pose2d(-60, 35, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Stopper stopper = new Stopper(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        // Build and execute the action
        Action driveAction = drive.actionBuilder(initialPose)
                .afterTime(0, () -> {shooterRotatorController.setTargetWorldAngle(55);})
                .stopAndAdd(shooter.setState(true))
                .strafeTo(new Vector2d(-27.5, 30))
                .afterTime(0, intake.setPower(1))

                .stopAndAdd(stopper.timedPower(1.0))
                .waitSeconds(2)
                .stopAndAdd(stopper.timedPower(-1.0))

                .splineToConstantHeading(new Vector2d(-1.5, 25), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(-1.5, 62.5), Math.toRadians(90.00))
                .strafeTo(new Vector2d(-27.5, 36.5))

                .stopAndAdd(stopper.timedPower(1.0))
                .waitSeconds(2)
                .stopAndAdd(stopper.timedPower(-1.0))

                .splineToConstantHeading(new Vector2d(23, 25), Math.toRadians(90.00))
                .splineToConstantHeading(new Vector2d(23, 68.5), Math.toRadians(90.00))
                .strafeTo(new Vector2d(26, 60))
                .strafeTo(new Vector2d(-26, 39.5))

                .stopAndAdd(stopper.timedPower(1.0))
                .afterTime(0, intake.setPower(1))
                .waitSeconds(2)
                .stopAndAdd(stopper.timedPower(-1.0))

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