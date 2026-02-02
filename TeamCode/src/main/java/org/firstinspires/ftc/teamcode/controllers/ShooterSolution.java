package org.firstinspires.ftc.teamcode.controllers;

public class ShooterSolution {
    public double bestAngle;
    public double targetRPM;
    public boolean reachable;

    public ShooterSolution(double angle, double rpm, boolean possible) {
        this.bestAngle = angle;
        this.targetRPM = rpm;
        this.reachable = possible;
    }



}
