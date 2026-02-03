package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoController {
    private double MaxPos = 1;
    private double ChangeValue = 0.001;
    private double  currentPos = 0;

    private Servo servo;

    public ServoController(HardwareMap hardwareMap, String servoName, Servo.Direction direction) {
        servo = hardwareMap.get(Servo.class, servoName);
        servo.setDirection(direction);
    }

    public void setPosition(double pos) {
        currentPos = pos;
        servo.setPosition(pos);
    }

    public void Increase() {
        if (currentPos + ChangeValue <= MaxPos) {
            currentPos += ChangeValue;
            servo.setPosition(currentPos);
        }
    }


    public void Decrease() {
        if (currentPos - ChangeValue >= 0) {
            currentPos -= ChangeValue;
            servo.setPosition(currentPos);
        }
    }

    public double getServoPosition() {
        return currentPos;
    }

}
