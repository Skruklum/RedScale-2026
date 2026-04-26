package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intake1Motor;
    private DcMotorEx intake2Motor;

    public Intake(HardwareMap hardwareMap) {
        intake1Motor = hardwareMap.get(DcMotorEx.class, "intake");
        intake2Motor = hardwareMap.get(DcMotorEx.class, "intake2");

        // 3. Set directions (usually one is reversed to work in tandem)
        intake1Motor.setDirection(DcMotor.Direction.REVERSE);
        intake2Motor.setDirection(DcMotor.Direction.FORWARD);

        intake1Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake2Motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public Action setPower(double power) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // 4. Update both motors inside the action
                intake1Motor.setPower(power);
                intake2Motor.setPower(power);
                return false;
            }
        };
    }
}