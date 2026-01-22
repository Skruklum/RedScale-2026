package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

@TeleOp(name = "Auto Lock Aim", group = "Test")

public class AutoLockAim extends  LinearOpMode {
    private Telemetry dashboardTelemetry;

    // --- HARDWARE ---
    private DcMotorEx turretMotor;

    // --- PID TUNING (THE MAGIC NUMBERS) ---
    public static PIDCoefficients coeffs = new PIDCoefficients(0.05, 0, 0.01);
    PIDFController pidController = new PIDFController(coeffs);

    // --- MANUAL TURN ---
    static final double TURRET_GEAR_RATIO = 4.75;
    private static final double CORE_HEX_TICKS_PER_REV =  (double) 288.0 * TURRET_GEAR_RATIO * 1/360;



    private IMU imu;

    private boolean InitRobotDegreeStatus = false;
    private double YawInitial = 0;
    private double RobotInitialDegree = 0;

    private boolean isTurretPIDActive = false;

    private boolean isCircleClicked = false;

    private double maxDeltaPower = 0.1; // Set the allowed power step-per-loop (tune value)
    private double previousPower = 0;    // Store the last applied motor power
    private double lastError = 0;
    private double power = 0;


    private double yawCurrentDegree = 0;
    private double robotCurrentDegree = 0;
    private double turretCurrentDegree = 0;

    private double YawTarget = 0;
    private double turretTargetYawDegree = 0;

    // Angle Constraints
    private static final double TURRET_MIN_ANGLE = -127;
    private static final double TURRET_MAX_ANGLE = 127;



    @Override
    public void runOpMode() {

        // Pendefinisian Motor sesuai config di Control Hub
//        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "left");
//        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "right");


        // Arah Motor (Sesuaikan jika robot jalannya terbalik)
//        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // --- FIX PENTING 1: Zero Power Behavior ---
//        // BRAKE: Agar roda langsung berhenti saat analog dilepas (tidak meluncur)
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Reset Encoders
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Mode Run
//        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // 1. HARDWARE INIT
        turretMotor = hardwareMap.get(DcMotorEx.class, "shooterRot");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);

        pidController.setOutputBounds(-1, 1);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry =  dashboard.getTelemetry();

        imu  = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);


        waitForStart();

        // 4. MAIN LOOP
        while (opModeIsActive()) {

            YawPitchRollAngles robotOrientation = imu.getRobotYawPitchRollAngles();

            if (!InitRobotDegreeStatus) {
                InitRobotDegreeStatus = true;
                YawInitial = robotOrientation.getYaw();
                RobotInitialDegree = getRobotDegree(YawInitial);
            }

            yawCurrentDegree = robotOrientation.getYaw() - YawInitial;
            robotCurrentDegree = getRobotDegree(yawCurrentDegree);
            turretCurrentDegree = turretMotor.getCurrentPosition() / CORE_HEX_TICKS_PER_REV;

            if (gamepad1.square) {
                YawTarget = robotOrientation.getYaw();
                turretTargetYawDegree = (turretCurrentDegree) +YawTarget;
                pidController.targetPosition = turretTargetYawDegree * CORE_HEX_TICKS_PER_REV;
            }


            if (gamepad1.circle && !isCircleClicked) {
                isTurretPIDActive = !isTurretPIDActive;
                isCircleClicked = true ;
            }else {
                isCircleClicked = false;
            }

            if (isTurretPIDActive) {

                double desiredPower = pidController.update((turretCurrentDegree + yawCurrentDegree) * CORE_HEX_TICKS_PER_REV);
//                double delta = desiredPower - previousPower;
//
//                if (Math.abs(delta) > maxDeltaPower) {
//                    desiredPower = previousPower + Math.signum(delta) * maxDeltaPower; // Increment by step size
//                }

                turretMotor.setPower(desiredPower); // Apply motor power
                previousPower = desiredPower; // Update the previous power
                lastError = pidController.lastError;
                power = desiredPower;
            }else {
                turretMotor.setPower(gamepad2.left_stick_y);
            }

            // Update to PID Control Logic
            if (isTurretPIDActive) {
                turretTargetYawDegree = constrainAngle(turretTargetYawDegree); // Constrain the target angle

                double shortestPathTarget = calculateShortestPath(turretCurrentDegree, turretTargetYawDegree);
                pidController.targetPosition = shortestPathTarget * CORE_HEX_TICKS_PER_REV; // Update the PID target position

                double desiredPower = pidController.update(turretCurrentDegree * CORE_HEX_TICKS_PER_REV);
//                double delta = desiredPower - previousPower;
//
//                if (Math.abs(delta) > maxDeltaPower) {
//                    desiredPower = previousPower + Math.signum(delta) * maxDeltaPower; // Increment by step size
//                }

//                turretMotor.setPower(desiredPower); // Apply motor power
                previousPower = desiredPower; // Update the previous power
                lastError = pidController.lastError;
                power = desiredPower;
            }else {
                turretMotor.setPower(gamepad2.left_stick_y);
            }



            telemetry.addLine("========================");
            telemetry.addData("pidController.targetPosition", pidController.targetPosition);
            telemetry.addData("pidController currentPosition", ((turretCurrentDegree + yawCurrentDegree) * CORE_HEX_TICKS_PER_REV));

            telemetry.addData("isTurretPIDActive", isTurretPIDActive);
            telemetry.addLine("========================");

            telemetry.addData("turret position", turretMotor.getCurrentPosition());
            telemetry.addData("turret degree", turretMotor.getCurrentPosition() / CORE_HEX_TICKS_PER_REV);
            telemetry.addData("turret degree relative to robot", getTurretDegreeRelativeToRobot(robotCurrentDegree, turretCurrentDegree));
            telemetry.addData("turret degree YAW relative to robot", turretCurrentDegree + yawCurrentDegree);
            telemetry.addData("turret degree YAW relative to robot (CONVERTED)", convertDegree(turretCurrentDegree + yawCurrentDegree));

            telemetry.addData("turret target degree", turretTargetYawDegree);

            telemetry.addData("Yaw", robotOrientation.getYaw());
            telemetry.addData("Robot Degree", getRobotDegree(robotOrientation.getYaw()));
            telemetry.addData("Robot Initial Degree", RobotInitialDegree);


            telemetry.addData("Pitch", robotOrientation.getPitch());
            telemetry.addData("Roll", robotOrientation.getRoll());
            telemetry.addData("power", power);
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);


            telemetry.update();

        }


    }

    // Constrain Angle Helper Function
    private double constrainAngle(double angle) {
        return Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, angle));
    }

    // Shortest Path Calculation
    private double calculateShortestPath(double currentAngle, double targetAngle) {
        double delta = (targetAngle - currentAngle + 180.0) % 360.0 - 180.0;
        return currentAngle + delta;
    }


    private double convertDegree(double degree) {
        if (degree < -180) {
            return 180 - ((180 + degree) * -1);
        }
        return degree;
    }

    private double getRobotDegree(double yaw) {
        if (yaw < 0) {
            return 360+yaw;
        }
        return yaw;
    }

    private  double getTurretDegreeRelativeToRobot(double robotDegree, double turretDegree) {
        double totalDegree = robotDegree + turretDegree;

        if (totalDegree > 360) {
            totalDegree -= 360;
        }

        return getRobotDegree(totalDegree);
    }

}