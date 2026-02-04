package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretAngle {
    private DcMotorEx turretMotor;


    public TurretAngle(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "shooterRot");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    public Action setState(boolean on) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (on) {
//                    shooterMotor.setVelocity(TARGET_VELO);
                } else {
//                    shooterMotor.setVelocity(0);
                }


                return false; // Action finishes immediately
            }
        };
    }

    // Roadrunner Action that blocks until RPM is within 5% of target

        };