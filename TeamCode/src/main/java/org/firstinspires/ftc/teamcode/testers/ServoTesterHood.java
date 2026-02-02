package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Tester Hood", group = "Test")
public class  ServoTesterHood extends OpMode {
    private Servo claw;

    private double clawPos = 0;
    private double limit = 0.6415;
    // max 0.37

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "shooterAd");
        claw.setDirection(Servo.Direction.FORWARD);
//        armPivot = hardwareMap.get(Servo.class, "armPivot");
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up && clawPos <= limit) {
            clawPos += 0.0005;
        }else if (gamepad1.dpad_down && clawPos >= 0) {
            clawPos -= 0.0005;
        }

        claw.setPosition(clawPos);

        telemetry.addData("shooterAd", clawPos);


        telemetry.update();
    }

}

// 12.03
// 33.5 / 37.5
// 22.5 / 28.5

// 23 degrees & 38 degress