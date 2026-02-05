package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterController {
    private double SHOOTER_MOTOR_COUNTS_PER_REV = 28.0;
    private double SHOOTER_HEIGHT = 35; // in cm

    private double SHOOTER_WHEEL_RADIUS = 4.5;
    private double SHOOTER_TICKS_PER_REV = 28;
    private double SHOOTER_EFFICIENCY = 0.85;
    private double SHOOTER_ANGLE_MINIMUM = 23.0;
    private double SHOOTER_ANGLE_MAXIMUM = 38.0;
    private double SHOOTER_ANGLE_MAXIMUM_POS = 0.6415;

    private boolean isActive = true;

    public static double P = 0;
    public static double F = 0.00017;
    public static PIDCoefficients coeffs = new PIDCoefficients(P, 0, 0.003, 0.0003, 0, 0.001);
    public static PIDFController pidfController = new PIDFController(coeffs, (d, v) -> {
        if (v != null) {
            return v*F;
        }
        return 0;
    });

    private DcMotorEx shooterMotor;

    public ShooterController() {

    }

    public ShooterController(HardwareMap hardwareMap, String shooterDeviceName) {
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidfController.setOutputBounds(-1, 1);
    }

    public void setInactive() {
        this.isActive = false;
    }

    public void setActive() {
        this.isActive = true;
    }


    public void setVelocity(double targetVelocity) {
        pidfController.targetVelocity = targetVelocity;
    }

    public void setPosition(double targetPosition) {
        pidfController.targetPosition = targetPosition;
    }

    public void update() {
       if (!isActive) {
           double power = pidfController.update(System.nanoTime(), 0, shooterMotor.getVelocity());
           shooterMotor.setPower(power);
       }
    }

    public ShooterSolution getBestShootingSolution(double distanceCm, double targetHeightCm) {
        double minRPM = Double.MAX_VALUE;
        double bestAngle = 38.0;
        boolean foundSolution = false;
        double MAX_ALLOWED_RPM = 4500.0;

        // Iterate through your mechanical range in 1-degree steps
        for (double angle = 23.0; angle <= 38.0; angle += 1.0) {
            double neededRPM = getTargetRPM(distanceCm, angle, targetHeightCm);

            // Check if this angle can hit the target under your 4500 RPM limit
            if (neededRPM > 1000 && neededRPM <= MAX_ALLOWED_RPM) {
                // We want the angle that requires the LOWEST RPM for maximum motor stability
                if (neededRPM < minRPM) {
                    minRPM = neededRPM;
                    bestAngle = angle;
                    foundSolution = true;
                }
            }
        }

        if (foundSolution) {
            return new ShooterSolution(bestAngle, minRPM, true);
        } else {
            // If unreachable, return your maximum mechanical capabilities as a fallback
            return new ShooterSolution(38.0, MAX_ALLOWED_RPM, false);
        }
    }

    public double getTargetRPM(double distanceCm, double angleDegrees, double targetHeightCm) {
        double g = 980.665; // cm/s^2
        double h0 = 20.0;    // Flywheel height in cm
        double angleRad = Math.toRadians(angleDegrees);

        // Physics formula to find launch velocity (v0)
        double numerator = g * Math.pow(distanceCm, 2);
        double denominator = 2 * Math.pow(Math.cos(angleRad), 2) *
                (distanceCm * Math.tan(angleRad) + h0 - targetHeightCm);

        if (denominator <= 0) return 0; // Target is physically unreachable at this angle

        double v0 = Math.sqrt(numerator / denominator);

        // Convert Launch Velocity to Motor RPM
        double circumference = 2 * Math.PI * SHOOTER_WHEEL_RADIUS;
        double rps = v0 / circumference;
        double rpm = (rps * 60) / SHOOTER_EFFICIENCY;

        return rpm;
    }


    public double angleToServo(double targetAngle) {
        // Calibration points
        double angleMin = SHOOTER_ANGLE_MINIMUM;
        double posMin = 0.0;
        double angleMax = SHOOTER_ANGLE_MAXIMUM;
        double posMax = SHOOTER_ANGLE_MAXIMUM_POS;

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

}
