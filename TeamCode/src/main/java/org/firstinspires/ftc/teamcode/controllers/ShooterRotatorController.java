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

    // --- CONSTANTS ---
    // Formula: (Encoder Ticks / Gear Ratio) / 360 degrees
    // Based on your code: ((256+261+265+260)/4)/90 = ~2.838 ticks per degree
    private static final double TICKS_PER_DEGREE = 2.838;

    public ShooterRotatorController(HardwareMap hardwareMap, RobotPoseController robotPoseController) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "shooterRot");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.robotPoseController = robotPoseController;

    }

    public void setTargetWorldAngle(double targetWorldAngle) {
        this.targetWorldAngle = targetWorldAngle;
    }

    public void update() {
        turretTicks = turretMotor.getCurrentPosition();
        turretDegrees = turretTicks / TICKS_PER_DEGREE;
        turretWorldAngle = normalizeAngle(robotPoseController.getRobotYaw() + turretDegrees);
    }

    public void activate() {

    }

    // Helper to keep angles between -180 and 180
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

}
