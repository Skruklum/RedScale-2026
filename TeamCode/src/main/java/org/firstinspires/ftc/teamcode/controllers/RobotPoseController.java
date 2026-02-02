package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotPoseController {
    private IMU imu;
    private double robotYaw = 0;
    private double yawOffset = 0;


    public  RobotPoseController(HardwareMap hardwareMap) {
        // IMU Init
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        yawOffset = getYaw();
    }

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    public void update() {
        robotYaw = normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - yawOffset);
    }

    public double getRobotYaw() {
        return robotYaw;
    }

    // Helper to keep angles between -180 and 180
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

}
