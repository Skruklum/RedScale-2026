package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    private DcMotorEx shooter1Motor;
    private DcMotorEx shooter2Motor;

    // Constants
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 5750;
    private static final double TARGET_VELO = (TARGET_RPM / 60.0) * TICKS_PER_REV;

    // Tuning constants - Adjust PID_F if motors don't reach 6000 RPM
    public static double PID_P = 20.0;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0;
    public static double PID_F = 14.5;
    PIDFCoefficients ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);

    public Shooter(HardwareMap hardwareMap) {
        shooter1Motor = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2Motor = hardwareMap.get(DcMotorEx.class, "shooter2");

        // Assuming motors face each other; one usually needs to be REVERSE
        // Adjust these based on your mechanical orientation
        shooter1Motor.setDirection(DcMotor.Direction.FORWARD);
        shooter2Motor.setDirection(DcMotor.Direction.REVERSE);

        // Apply PIDF for velocity control
        shooter1Motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);
        shooter2Motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);

        // FLOAT prevents jerky stops which can damage gears/belts
        shooter1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooter2Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    // Action to turn both motors ON or OFF
    public Action setState(boolean on) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (on) {
                    shooter1Motor.setVelocity(TARGET_VELO);
                    shooter2Motor.setVelocity(TARGET_VELO);
                } else {
                    shooter1Motor.setVelocity(0);
                    shooter2Motor.setVelocity(0);
                }
                return false;
            }
        };
    }

    // Blocks the sequence until BOTH motors are at 95% speed
    public Action waitUntilReady() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double rpm1 = (shooter1Motor.getVelocity() * 60.0) / TICKS_PER_REV;
                double rpm2 = (shooter2Motor.getVelocity() * 60.0) / TICKS_PER_REV;

                packet.put("Shooter 1 RPM", String.format("%.0f", rpm1));
                packet.put("Shooter 2 RPM", String.format("%.0f", rpm2));

                // Returns true to keep waiting if either motor is too slow
                return (rpm1 < (TARGET_RPM * 0.95)) || (rpm2 < (TARGET_RPM * 0.95));
            }
        };
    }
}