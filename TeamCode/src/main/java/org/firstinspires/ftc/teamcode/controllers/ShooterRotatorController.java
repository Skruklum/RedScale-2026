package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterRotatorController {
    private DcMotorEx turretMotor;

    private double turretTicks = 0;
    private double turretDegrees = 0;
    private double turretWorldAngle = 0;
    private double targetWorldAngle = 0;

    private RobotPoseController robotPoseController;

    // Mechanical Limits (Degrees relative to robot front)
    private static final double MAX_TURRET_ANGLE_POSITIVE = 170; // Left Limit
    private static final double MAX_TURRET_ANGLE_NEGATIVE = -150; // Right Limit

    // --- CONSTANTS ---
    // Formula: (Encoder Ticks / Gear Ratio) / 360 degrees
    // Based on your code: ((256+261+265+260)/4)/90 = ~2.838 ticks per degree
    private static final double TICKS_PER_DEGREE = 2.838;

    private boolean isAtLimit = false;

    private double previousError = 0;

    // --- PID TUNING ---
    // Start with P=0.03. Increase if sluggish. D=0.001 stops oscillation.
    public static PIDCoefficients coeffs = new PIDCoefficients(0.03, 0, 0.001);
    PIDFController pidController = new PIDFController(coeffs);

    public ShooterRotatorController(HardwareMap hardwareMap, RobotPoseController robotPoseController, String shooterRotName) {
        turretMotor = hardwareMap.get(DcMotorEx.class, shooterRotName);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.robotPoseController = robotPoseController;

    }

    public void setTargetWorldAngle() {
        this.targetWorldAngle = turretWorldAngle;
    }

    public void setTargetWorldAngle(double angle) {
        this.targetWorldAngle = angle;
    }

    public void update() {
        turretTicks = turretMotor.getCurrentPosition();
        turretDegrees = turretTicks / TICKS_PER_DEGREE;
        turretWorldAngle = normalizeAngle(robotPoseController.getRobotYaw() + turretDegrees);
    }

    public double getTurretWorldAngle() {
        return turretWorldAngle;
    }

    public void activate() {
        // Step A: Calculate 'Ideal' Angle relative to Robot
        // "Ideally, the turret should look X degrees to the left/right to see the target"
        double requiredTurretAngle = normalizeAngle(targetWorldAngle - robotPoseController.getRobotYaw());
//        double requiredTurretAngle = error;

        // Step B: Clamp to Mechanical Limits
        double clampedTargetAngle;

        if (requiredTurretAngle > MAX_TURRET_ANGLE_POSITIVE) {
            // Robot turned too far right -> Hold Left Limit
            clampedTargetAngle = MAX_TURRET_ANGLE_POSITIVE;
            isAtLimit = true;
        } else if (requiredTurretAngle < MAX_TURRET_ANGLE_NEGATIVE) {
            // Robot turned too far left -> Hold Right Limit
            clampedTargetAngle = MAX_TURRET_ANGLE_NEGATIVE;
            isAtLimit = true;
        } else {
            // Within range -> Track perfectly
            clampedTargetAngle = requiredTurretAngle;
            isAtLimit = false;
        }

        // Step C: Execute PID
        // Convert Degrees back to Ticks for the PID controller
        double targetTicks = clampedTargetAngle * TICKS_PER_DEGREE;

        pidController.targetPosition = targetTicks;
        double power = pidController.update(turretTicks);

        turretMotor.setPower(power);

        previousError = requiredTurretAngle;


    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public double getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    public boolean IsAtLimit() {
        return isAtLimit;
    }

    // Helper to keep angles between -180 and 180
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

}
