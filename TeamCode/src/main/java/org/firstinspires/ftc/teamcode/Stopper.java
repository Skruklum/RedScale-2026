package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Stopper {
    private CRServo stopperServo;

    public Stopper(HardwareMap hardwareMap) {
        stopperServo = hardwareMap.get(CRServo.class, "stopper");
        stopperServo.setDirection(DcMotor.Direction.FORWARD);
    }


    public Action setPower(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stopperServo.setPower(power);
                return false; // Returns false because it's a "one-and-done" action
            }
        };
    }
}
