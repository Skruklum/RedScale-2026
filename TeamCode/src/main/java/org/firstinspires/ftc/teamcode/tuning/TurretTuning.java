package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;

import java.lang.reflect.Array;

@Config
@TeleOp(group = "Tuning", name = "Turret Tuning")
public class TurretTuning extends OpMode {
    private DcMotorEx Motor;

    private int currentSetpointIndex = -1;
    private int currentSetpoint = -1;

    private boolean isAClicked = false;
    private final int[] setpoints = {0, 132, 325, -132, -325, 500, -500};


    public static PIDCoefficients coeffs = new PIDCoefficients(0.05, 0, 0.1);
    PIDFController pidController = new PIDFController(coeffs);

    private Telemetry dashboardTelemetry;

    @Override
    public void init() {
        Motor = hardwareMap.get(DcMotorEx.class, "shooterRot");
        Motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Motor.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addData("power",0);
        telemetry.addData("error",0);

        telemetry.addData("Setpoint",0 );
        telemetry.addData("Positions",0);
        pidController.setOutputBounds(-1, 1);

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
                pidController.targetPosition = currentSetpoint;
                currentSetpointIndex = 0;
            }
            return;
        }

        if (gamepad1.a) {
            if (!isAClicked) {
                if (currentSetpointIndex == setpoints.length-1) {
                    currentSetpointIndex = 0;
                }else {
                    currentSetpointIndex += 1;
                }
                currentSetpoint = (int) Array.get(setpoints, currentSetpointIndex);
                pidController.targetPosition = currentSetpoint;
                isAClicked = true;
            }
        }else {
            isAClicked = false;
        }

        double power = pidController.update(Motor.getCurrentPosition());
        Motor.setPower(power);

        dashboardTelemetry.addData("error",pidController.lastError);

        dashboardTelemetry.addData("power",power);
        dashboardTelemetry.addData("Setpoint",currentSetpoint );
        dashboardTelemetry.addData("Positions",Motor.getCurrentPosition() );

        dashboardTelemetry.update();

        telemetry.addData("error",pidController.lastError);

        telemetry.addData("power",power);
        telemetry.addData("Setpoint",currentSetpoint );
        telemetry.addData("Positions",Motor.getCurrentPosition() );

        telemetry.update();
    }

}