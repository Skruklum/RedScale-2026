package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Stopper {
    private CRServo stopperServo;

    public Stopper(HardwareMap hardwareMap) {
        stopperServo = hardwareMap.get(CRServo.class, "stopper");
        stopperServo.setDirection(DcMotor.Direction.FORWARD);
    }

    public Action timedPower(double power) {
        return new Action() {
            private long startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) {
                    startTime = System.currentTimeMillis();
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (elapsedTime < 500) {
                    stopperServo.setPower(power);
                    packet.put("Stopper Status", power > 0 ? "Forward" : "Reverse");
                    packet.put("Stopper Time Left", (500 - elapsedTime) / 1000.0);
                    return true; // Keep running
                } else {
                    stopperServo.setPower(0); // Shut off
                    packet.put("Stopper Status", "OFF");
                    return false; // Action finished
                }
            }
        };
    }

    public Action setPower(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stopperServo.setPower(power);
                return false;
            }
        };
    }
}