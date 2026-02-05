package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name ="Test Yaw", group = "Test")

public class testYaw extends OpMode {
    private IMU imu;


    @Override
    public void init() {

        // IMU Init
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
    }

    @Override
    public void loop() {
        /* ================== CALCULATE ANGLES ================== */
        // Robot Heading (World Frame)
        double robotYaw = normalizeAngle(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );

        telemetry.addData("Robot Yaw (IMU)", "%.2f", robotYaw);

    }

    /* ================== HELPERS ================== */
    private double normalizeAngle(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }
}
