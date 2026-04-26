package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeIndividual { // Class name MUST match file name
    private final DcMotorEx intake1Motor;
    private final DcMotorEx intake2Motor;

    public IntakeIndividual(HardwareMap hardwareMap) {
        intake1Motor = hardwareMap.get(DcMotorEx.class, "intake");
        intake2Motor = hardwareMap.get(DcMotorEx.class, "intake2");

        intake1Motor.setDirection(DcMotor.Direction.REVERSE);
        intake2Motor.setDirection(DcMotor.Direction.FORWARD);

        intake1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake2Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Controls a specific motor individually
     * @param motorIndex 1 or 2
     * @param power Power level (-1.0 to 1.0)
     */
    public Action setIndividualPower(int motorIndex, double power) {
        return packet -> {
            if (motorIndex == 1) {
                intake1Motor.setPower(power);
            } else if (motorIndex == 2) {
                intake2Motor.setPower(power);
            }
            return false;
        };
    }

    /**
     * Controls both motors at once
     */
    public Action setAllPower(double power) {
        return packet -> {
            intake1Motor.setPower(power);
            intake2Motor.setPower(power);
            return false;
        };
    }

    /**
     * Activates Intake 1, waits, then activates Intake 2
     * @param power The power to set for both
     * @param delaySeconds How long to wait before starting motor 2
     */
    public Action stagedIntake(double power, double delaySeconds) {
        return new Action() {
            private double startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // If this is the first time running, record the time
                if (startTime == 0) {
                    startTime = System.currentTimeMillis() / 1000.0;
                    intake2Motor.setPower(power); // Start motor 2 immediately
                }

                double currentTime = System.currentTimeMillis() / 1000.0;

                // Once the delay has passed, start motor 2
                if (currentTime - startTime >= delaySeconds) {
                    intake1Motor.setPower(power);
                    return false; // Action is finished
                }

                return true; // Keep running this action until the time is up
            }
        };


    }
}