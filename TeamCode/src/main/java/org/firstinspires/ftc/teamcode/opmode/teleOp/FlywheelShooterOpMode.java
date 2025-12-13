package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoTunePIDF;

@TeleOp(name = "FlywheelShooterOpMode", group = "Testing")
public class FlywheelShooterOpMode extends LinearOpMode {

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;
    private VoltageSensor voltageSensor;

    private AutoTunePIDF pidf;

    private final double TICKS_PER_REV = 28.0;
    private final double TARGET_VELOCITY = 3000.0;
    private final double RELAY_AMPLITUDE = 0.3;

    private boolean isAutotuning = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new AutoTunePIDF();
        pidf.setNominalVoltage(12.0);
        pidf.setOutputLimits(-1.0, 1.0);

        telemetry.addData("Status", "Initialized. Press PLAY to start.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double batteryVoltage = voltageSensor.getVoltage();
            double currentVelocity = (flywheelLeft.getVelocity() + flywheelRight.getVelocity()) / 2.0;

            if (gamepad1.y && !isAutotuning) {
                pidf.startAutoTune(RELAY_AMPLITUDE);
                isAutotuning = true;
                telemetry.addData("Status", "Autotuning started...");
            } else if (gamepad1.a) {
                pidf.setSetpoint(TARGET_VELOCITY);
                telemetry.addData("Status", "Targeting velocity: " + TARGET_VELOCITY);
            }

            double power = pidf.update(currentVelocity, batteryVoltage);
            flywheelLeft.setPower(power);
            flywheelRight.setPower(power);

            if (isAutotuning && pidf.isTuningComplete()) {
                isAutotuning = false;
                double kp = pidf.getKp();
                double ki = pidf.getKi();
                double kd = pidf.getKd();
                telemetry.addData("Tuning Complete", "Kp: %.4f, Ki: %.4f, Kd: %.4f", kp, ki, kd);
            }

            telemetry.addData("Current Velocity", "%.2f ticks/sec", currentVelocity);
            telemetry.addData("Setpoint", "%.2f", pidf.setpoint);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            telemetry.addData("Status", isAutotuning ? "Autotuning..." : "Running");
            telemetry.update();
        }

        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }
}
