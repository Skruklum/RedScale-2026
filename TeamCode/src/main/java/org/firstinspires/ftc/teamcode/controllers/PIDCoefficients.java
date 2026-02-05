package org.firstinspires.ftc.teamcode.controllers;


public final class PIDCoefficients {
    public double kP, kI, kD, kV, kA, kStatic;
    public PIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public PIDCoefficients(double kP, double kI, double kD, double kV, double kA, double kStatic) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;

    }
}