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
        telemetry.addData("SHOOTER Velocity", rpmToVelocityCmS(SHOOTER_RPM, SHOOTER_WHEEL_RADIUS));

        telemetry.addData("SHOOTER TARGET ANGLE", currentShooterAngle);
        telemetry.addData("SHOOTER TARGET ANGLE POS", angleToServo(currentShooterAngle));
        telemetry.addData("SHOOTER TARGET DISTANCE", currentShootingDistance);
        shooterAd.setPosition(angleToServo(currentShooterAngle));



        if (shooterActive) {


            shooterMotor.setPower(1);
        } else {
            telemetry.addLine("=========================");
            telemetry.addLine("getTargetRPM by distance & angle");

            double taretRPM = getTargetRPM(currentShootingDistance, currentShooterAngle, 98.5);
            double targetVelocityTicks = rpmToVelocityTicks(taretRPM);
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

    private double doubleIncToCM(double inch) {
        return inch * 2.54;
    }
    public double angleToServo(double targetAngle) {
        // Calibration points
        double angleMin = 23.0;
        double posMin = 0.0;
        double angleMax = 38.0;
        double posMax = 0.6415;

        // Linear Interpolation Formula:
        // y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double position = posMin + (targetAngle - angleMin) * (posMax - posMin) / (angleMax - angleMin);

        // Clamp the value to ensure it stays within your mechanical limits [0, 0.6415]
        if (position < posMin) return posMin;
        if (position > posMax) return posMax;

        return position;
    }


    // Convert flywheel RPM to velocity ticks
    public double rpmToVelocityTicks(double rpm) {
        return rpm /  60 * SHOOTER_MOTOR_COUNTS_PER_REV;
    }

    // Convert flywheel RPM to surface velocity in cm/s
    public double rpmToVelocityCmS(double rpm, double wheelRadiusCm) {
        double circumference = 2 * Math.PI * wheelRadiusCm; // cm
        double revPerSecond = rpm / 60.0;
        return circumference * revPerSecond;
    }

    public double getTargetRPM(double distanceCm, double angleDegrees, double targetHeightCm) {
        double g = 980.665; // Gravity in cm/s^2
        double h0 = SHOOTER_HEIGHT;    // Flywheel height in cm
        double angleRad = Math.toRadians(angleDegrees);

        // 1. Calculate required launch velocity (v0) in cm/s
        double numerator = g * Math.pow(distanceCm, 2);
        double denominator = 2 * Math.pow(Math.cos(angleRad), 2) *
                (distanceCm * Math.tan(angleRad) + h0 - targetHeightCm);

        // If target is physically unreachable at this angle, return 0
        if (denominator <= 0) return 0;
        double v0 = Math.sqrt(numerator / denominator);

        // 2. Convert Velocity (cm/s) to RPM
        // Formula: RPM = (v0 / (2 * PI * R)) * 60 / Efficiency
        double circumference = 2 * Math.PI * SHOOTER_WHEEL_RADIUS;
        double rps = v0 / circumference;
        double targetRPM = (rps * 60) / SHOOTER_EFFICIENCY;

        // 3. Cap at your motor's 4500 RPM max
        return Math.min(targetRPM, 4500.0);
    }

    // Get required launch angle (degrees) at a fixed velocity and distance
    public double getRequiredAngleDeg(double distanceCm,
                                      double targetHeightCm,
                                      double launchVelocityCmS) {
        double g = 980.665; // cm/s^2
        double h0 = SHOOTER_HEIGHT;   // launcher height in cm
        double x = distanceCm;
        double y = targetHeightCm - h0;
        double v = launchVelocityCmS;

        double termInsideSqrt =
                Math.pow(v, 4) - g * (g * x * x + 2 * y * v * v);

        // unreachable if negative
        if (termInsideSqrt < 0) {
            return -1; // signal "no solution"
        }

        // low (flatter) trajectory solution
        double sqrtTerm = Math.sqrt(termInsideSqrt);
        double tanTheta = (v * v - sqrtTerm) / (g * x);
        double angleRad = Math.atan(tanTheta);

        return Math.toDegrees(angleRad);
    }

    /**
     * Converts motor encoder ticks per second into physical launch velocity (cm/s).
     * @param currentTPS The value from motor.getVelocity().
     * @param ticksPerRev The TPR of your motor (e.g., 28 for 1:1 gear ratio).
     * @return The estimated launch velocity in cm/s.
     */
    public double ticksToVelocityCmS(double currentTPS, double ticksPerRev) {
        // 1. Calculate how many rotations the motor is doing per second
        double rotationsPerSecond = currentTPS / ticksPerRev;

        // 2. Calculate the circumference of the flywheel (cm)
        double circumference = 2 * Math.PI * SHOOTER_WHEEL_RADIUS;

        // 3. Physical surface velocity (cm/s)
        double surfaceVelocity = rotationsPerSecond * circumference;

        // 4. Adjust for efficiency/slip (estimated 85%)
        // This represents the actual speed the ball leaves the shooter
        double efficiencyFactor = 0.85;

        return surfaceVelocity * efficiencyFactor;
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

