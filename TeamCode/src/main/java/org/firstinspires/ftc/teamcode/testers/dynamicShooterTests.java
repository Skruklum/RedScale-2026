package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.ShooterController;

import java.lang.reflect.Array;

@TeleOp(name = "Dynamic Shooting Test", group = "Test")

public class dynamicShooterTests  extends OpMode {

    private DcMotorEx shooterMotor; // 0.6415


    private boolean gamepad2_isXClicked = false;

    boolean shooterActive = false; // False = Manual, True = Auto-Lock
    private double SHOOTER_MOTOR_COUNTS_PER_REV = 28.0;
    private double SHOOTER_HEIGHT = 35; // in cm

    private double SHOOTER_WHEEL_RADIUS = 4.5;
    private double SHOOTER_TICKS_PER_REV = 28;
    private double SHOOTER_EFFICIENCY = 0.85;
    private double SHOOTER_ANGLE_MINIMUM = 23.0;
    private double SHOOTER_ANGLE_MAXIMUM = 38.0;
    private double SHOOTER_ANGLE_MAXIMUM_POS = 0.6415;
    private CRServo stopperS;


    private boolean isSquareClicked = false;
    private final int[] shooterAnles = {23, 30, 35, 38};
    private int currentShooterAnleIndex = -1;
    private int currentShooterAngle = -1;

    private boolean isCicleClicked = false;
    private final int[] shootingDistances = {30, 50, 100, 150};
    private int currentShootingDistanceIndex = -1;
    private int currentShootingDistance = -1;

    private boolean isTriangleClicked = false;


    private DcMotorEx frontLeft, frontRight, backLeft, backRight, intake;
    private Servo shooterAd;
    private ShooterController shooterController;


    @Override
    public void init() {

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterAd = hardwareMap.get(Servo.class, "shooterAd");

        frontLeft   = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight  = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft    = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight   = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        stopperS    = hardwareMap.get(CRServo.class, "stopper");
        intake      = hardwareMap.get(DcMotorEx.class, "intake");


        // ---------------- MOTOR DIRECTIONS ----------------
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients currentPIDF = new PIDFCoefficients(20, 0, 0, 14.5);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentPIDF);

        shooterController = new ShooterController();

        telemetry.addLine("Finish Initializing Camera..");
        telemetry.update();

    }

    @Override
    public void init_loop() {




        telemetry.update();

    }


    @Override
    public void start() {
        telemetry.addLine("Press (TRIANGLE) to begin");


        telemetry.update();
    }

    @Override
    public void loop() {

        // ===== INTAKE =====
        if (gamepad1.right_bumper) intake.setPower(1);
        else if (gamepad1.left_bumper) intake.setPower(-1);
        else intake.setPower(0);

        // ===== DRIVE =====
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double max = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
        frontLeft.setPower((y + x + rx) / max);
        frontRight.setPower((y - x - rx) / max);
        backLeft.setPower((y - x + rx) / max);
        backRight.setPower((y + x - rx) / max);



        if (!isTriangleClicked && currentShooterAnleIndex == -1) {
            if (gamepad1.triangle) {
                currentShootingDistance = (int) Array.get(shootingDistances, 0);
                currentShootingDistanceIndex = 0;

                currentShooterAngle = (int) Array.get(shooterAnles, 0);
                currentShooterAnleIndex = 0;
            }
            return;
        }

        if (gamepad1.circle) {
            if (!isCicleClicked) {
                if (currentShootingDistanceIndex == shootingDistances.length-1) {
                    currentShootingDistanceIndex = 0;
                }else {
                    currentShootingDistanceIndex += 1;
                }
                currentShootingDistance = (int) Array.get(shootingDistances, currentShootingDistanceIndex);
                isCicleClicked = true;
            }
        }else if (!gamepad1.circle) {
            isCicleClicked = false;
        }


        if (gamepad1.square) {
            if (!isSquareClicked) {
                if (currentShooterAnleIndex == shooterAnles.length-1) {
                    currentShooterAnleIndex = 0;
                }else {
                    currentShooterAnleIndex += 1;
                }
                currentShooterAngle = (int) Array.get(shooterAnles, currentShooterAnleIndex);
                isSquareClicked = true;
            }
        }else if (!gamepad1.square){
            isSquareClicked = false;
        }


        // --- SHOOTER LOGIC (X Button) ---
        if (gamepad2.cross && !gamepad2_isXClicked) {
            shooterActive = !shooterActive;
            gamepad2_isXClicked = true;
        }else if (!gamepad2.cross) {
            gamepad2_isXClicked = false;
        }



        // ===== STOPPER =====
        if (gamepad2.dpad_right) stopperS.setPower(1);
        else if (gamepad2.dpad_left) stopperS.setPower(-1);
        else stopperS.setPower(0);

//        shooterAd.setPosition(convertAngleToShootingPos(currentShooterAngle));

        double SHOOTER_RPM = shooterMotor.getVelocity() / SHOOTER_MOTOR_COUNTS_PER_REV * 60;

        telemetry.addData("SHOOTER VELOCITY", shooterMotor.getVelocity());
        telemetry.addData("SHOOTER RPM", SHOOTER_RPM);
        telemetry.addData("SHOOTER Velocity", shooterController.rpmToVelocityCmS(SHOOTER_RPM, SHOOTER_WHEEL_RADIUS));

        telemetry.addData("SHOOTER TARGET ANGLE", currentShooterAngle);
        telemetry.addData("SHOOTER TARGET ANGLE POS", shooterController.angleToServo(currentShooterAngle));
        telemetry.addData("SHOOTER TARGET DISTANCE", currentShootingDistance);
        shooterAd.setPosition(shooterController.angleToServo(currentShooterAngle));



        if (shooterActive) {
            shooterMotor.setPower(1);
        } else {
            telemetry.addLine("=========================");
            telemetry.addLine("getTargetRPM by distance & angle");

            double taretRPM = shooterController.getTargetRPM(currentShootingDistance, currentShooterAngle, 98.5);
            double targetVelocityTicks = shooterController.rpmToVelocityTicks(taretRPM);
            telemetry.addData("SHOOTER TARGET RPM", taretRPM);
            telemetry.addData("SHOOTER VELOCITY TICKS", targetVelocityTicks);
            shooterMotor.setVelocity(targetVelocityTicks);



        }



        telemetry.addLine("=========================");

//        telemetry.addLine("getTargetRPM by distance & angle");
//
//        telemetry.addLine("=========================");


        telemetry.addData("gamepad2.cross", gamepad2.cross);
        telemetry.addData("shooterActive", shooterActive);

        telemetry.update();


    }

    @Override
    public void stop() {

    }


    /**
     * Converts a measurement from inches to centimeters.
     * @param inch The value in inches.
     * @return The value converted to centimeters.
     */
    public double inchToCm(double inch) {
        return inch * 2.54;
    }

}

