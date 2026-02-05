
package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;


@Config
@TeleOp(group = "Tuning", name = "Shooting Tuning Custom PID")
public class ShootingTuningCustomPID extends OpMode {
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


    public static double P = 0;
    public static double F = 0.00017;


    public static PIDCoefficients coeffs = new PIDCoefficients(P, 0, 0.003, 0.0003, 0, 0.001);
    public static PIDFController pidfController = new PIDFController(coeffs, (d, v) -> {
        if (v != null) {
            return v*F;
        }
        return 0;
    });

    private Telemetry dashboardTelemetry;

    @Override
    public void init() {
        Motor = hardwareMap.get(DcMotorEx.class, "shooter");

        telemetry.addData("power",0);
        telemetry.addData("error",0);

        telemetry.addData("Setpoint",0 );
        telemetry.addData("Positions",0);

        Motor.setDirection(DcMotor.Direction.REVERSE);
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidfController.setOutputBounds(-1, 1);

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


        pidfController.targetVelocity = currentSetpoint;

        double power = pidfController.update(System.nanoTime(), 0, Motor.getVelocity());

        Motor.setPower(power);

        if (gamepad1.right_bumper)  {
            Motor.setPower(1);
        }else if (!gamepad1.right_bumper && power == 0){
            Motor.setPower(0);
        }

        double SHOOTER_RPM = Motor.getVelocity() / SHOOTER_MOTOR_COUNTS_PER_REV * 60;
        double error = currentSetpoint - Motor.getVelocity();


        dashboardTelemetry.addData("error",error );

        dashboardTelemetry.addData("TARGET VELOCITY",pidfController.targetVelocity );
        dashboardTelemetry.addData("CURRENT VELOCITY",Motor.getVelocity() );
        dashboardTelemetry.addData("SHOOTER POWER NEEDD",power );

        dashboardTelemetry.addData("SHOOTER_RPM",SHOOTER_RPM );
        dashboardTelemetry.addData("kV",pidfController.getKV() );


        dashboardTelemetry.update();

//        telemetry.addData("error", Motor.getController().);

//        telemetry.addData("power",power);
        telemetry.addData("Setpoint",currentSetpoint );
        telemetry.addData("STEP", stepSizes[stepIndex] );

        telemetry.addData("Positions",Motor.getCurrentPosition() );

        telemetry.update();
    }

}