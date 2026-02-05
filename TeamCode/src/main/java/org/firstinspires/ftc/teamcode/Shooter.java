package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    private DcMotorEx shooterMotor;

    // Constants for HD Hex (No Gearbox)
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM = 2950.0;
    // (3000 / 60) * 28 = 1400 ticks/sec
    private static final double TARGET_VELO = (TARGET_RPM / 60.0) * TICKS_PER_REV;

    // ---------------- SHOOTER PIDF ----------------

    public static double PID_P = 20.0;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0;
    public static double PID_F = 14.5;
    PIDFCoefficients ShooterPIDF = new PIDFCoefficients(PID_P, PID_I, PID_D, PID_F);

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);

        // Essential for setVelocity() to work
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterPIDF);


        // Use FLOAT for shooters to prevent mechanical shock when stopping
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    // Roadrunner Action to toggle the motor
    public Action setState(boolean on) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (on) {
                    shooterMotor.setVelocity(TARGET_VELO);
                } else {
                    shooterMotor.setVelocity(0);
                }


                return false; // Action finishes immediately
            }
        };
    }

    // Roadrunner Action that blocks until RPM is within 5% of target
    public Action waitUntilReady() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double currentVelo = shooterMotor.getVelocity();
                double currentRPM = (currentVelo * 60.0) / 28.0;

                // These will show up on your Driver Hub while it waits
                packet.put("Live RPM", String.format("%.0f", currentRPM));
                packet.put("Status", "Waiting for spin-up...");

                // Keeps running (returning true) until we are at 95% of 3000 RPM
                return currentRPM < (TARGET_RPM * 0.95);
            }
        };
    }
}