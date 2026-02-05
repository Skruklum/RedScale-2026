package org.firstinspires.ftc.teamcode.testers;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Shooter Tests", group = "Test")

public class shooterTests extends OpMode {

    private DcMotorEx  shooterMotor; // 0.6415


    private boolean gamepad2_isXClicked = false;

    boolean shooterActive = false; // False = Manual, True = Auto-Lock
   private double MotorCountsPerRev = 28.0; // False = Manual, True = Auto-Lock

    @Override
    public void init() {

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Finish Initializing Camera..");
        telemetry.update();

    }

    @Override
    public void init_loop() {




        telemetry.update();

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {


        // --- SHOOTER LOGIC (X Button) ---
        if (gamepad2.cross && !gamepad2_isXClicked) {
            shooterActive = !shooterActive;
            gamepad2_isXClicked = true;
        }else if (!gamepad2.cross) {
            gamepad2_isXClicked = false;
        }

        if (shooterActive) {


            shooterMotor.setPower(1);
        } else {
            shooterMotor.setPower(0);

        }

        telemetry.addData("SHOOTER VELOCITY", shooterMotor.getVelocity());
        telemetry.addData("SHOOTER RPM", shooterMotor.getVelocity() / MotorCountsPerRev * 60);

        telemetry.addData("gamepad2.cross", gamepad2.cross);
        telemetry.addData("shooterActive", shooterActive);

        telemetry.update();


    }

    @Override
    public void stop() {

    }

}


