package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;


@Config
@TeleOp(group = "Tuning", name = "Shooting Tuning")
public class ShootingTuning extends OpMode {
    private DcMotorEx Motor;

    private int currentSetpointIndex = -1;
    private int currentSetpoint = -1;

    private boolean isAClicked = false;
    private boolean isSquareClicked = false;

    private final int[] setpoints = {0, 100, 500, 1000, 2000, 1500, 1000, 500, 250};

    private final double[] stepSizes = {0.0001, 0.001, 0.01, 0.1, 1, 10};
    private int stepIndex = -1;


    private double SHOOTER_MOTOR_COUNTS_PER_REV = 28.0;

    private double highVelocity = 2100;
    private double lowVelocity = 500;


    public static double P = 60;
    public static double F = 17.5;


    public static PIDFCoefficients coeffs = new PIDFCoefficients(P,0,0,F);

    private Telemetry dashboardTelemetry;

    @Override
    public void init() {
        Motor = hardwareMap.get(DcMotorEx.class, "shooter");



        telemetry.addData("power",0);
        telemetry.addData("error",0);

        telemetry.addData("Setpoint",0 );
        telemetry.addData("Positions",0);

        Motor.setDirection(DcMotor.Direction.REVERSE);

        Motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry =  dashboard.getTelemetry();
    }

    @Override
    public void start() {
        telemetry.addLine("Press (A) to begin");
        telemetry.addData("power",0);
        telemetry.addData("error",0);

        telemetry.addData("Setpoint",0 );
        telemetry.addData("Positions",0);

        telemetry.update();
    }

    public void loop() {
        if (!isAClicked && currentSetpointIndex == -1) {
            if (gamepad1.a) {
                currentSetpoint = (int) Array.get(setpoints, 0);
                currentSetpointIndex = 0;
                stepIndex = 0;
            }
            return;
        }

        if (gamepad1.square) {
            if (!isSquareClicked) {
                if (stepIndex == stepSizes.length-1) {
                    stepIndex = 0;
                }else {
                    stepIndex += 1;
                }
                isSquareClicked = true;
            }
        }else if (!gamepad1.square) {
            isSquareClicked = false;
        }

        if (gamepad1.cross) {
            if (!isAClicked) {
                if (currentSetpointIndex == setpoints.length-1) {
                    currentSetpointIndex = 0;
                }else {
                    currentSetpointIndex += 1;
                }
                currentSetpoint = (int) Array.get(setpoints, currentSetpointIndex);
                isAClicked = true;
            }
        }else if (!gamepad1.cross) {
            isAClicked = false;
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed())  {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed())  {
            P -= stepSizes[stepIndex];
        }

        coeffs = new PIDFCoefficients(P,0,0,F);
        Motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);

        Motor.setVelocity(currentSetpoint);

        double SHOOTER_RPM = Motor.getVelocity() / SHOOTER_MOTOR_COUNTS_PER_REV * 60;
        double error = currentSetpoint - Motor.getCurrentPosition();


        dashboardTelemetry.addData("error",error );

        dashboardTelemetry.addData("Setpoint",currentSetpoint );
        dashboardTelemetry.addData("SHOOTER_RPM",SHOOTER_RPM );
        dashboardTelemetry.addData("VELOCITY",Motor.getVelocity() );


        dashboardTelemetry.update();

//        telemetry.addData("error", Motor.getController().);

//        telemetry.addData("power",power);
        telemetry.addData("Setpoint",currentSetpoint );
        telemetry.addData("STEP", stepSizes[stepIndex] );

        telemetry.addData("Positions",Motor.getCurrentPosition() );

        telemetry.update();
    }

}