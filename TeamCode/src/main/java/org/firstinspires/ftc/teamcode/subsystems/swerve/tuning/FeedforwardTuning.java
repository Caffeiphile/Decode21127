package org.firstinspires.ftc.teamcode.subsystems.swerve.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Feedforward Tuning", group="Tuning")
public class FeedforwardTuning extends OpMode {
    private SwerveModuleHardware hardware;
    private DriveMotorController controller;
    private ElapsedTime testTimer;

    private double kV = SwerveConstants.DRIVE_KV;
    private double kA = SwerveConstants.DRIVE_KA;
    private double kS = SwerveConstants.DRIVE_KS;

    private enum TestState {
        IDLE,
        STATIC_TEST,
        VELOCITY_TEST,
        ACCEL_TEST,
        COMPLETE
    }

    private TestState state = TestState.IDLE;
    private int velocityTestStep = 0;
    private double[] testVelocities = {0.5, 1.0, 1.5, 2.0};
    private double[] measuredPowers = new double[4];

    private double targetVelocity = 0;
    private double targetAccel = 0;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void init() {
        testTimer = new ElapsedTime();

        hardware = new SwerveModuleHardware(
                hardwareMap,
                "kraiTop",
                "kraiBottom",
                "kraiEncoder",
                true,
                true,
                -0.0952
        );

        controller = new DriveMotorController(kV, kA, kS);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Instructions", "Press A to start characterization");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        boolean xPressed = gamepad1.x && !lastX;
        boolean yPressed = gamepad1.y && !lastY;

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        if (gamepad1.dpadUpWasPressed()) kV += 0.01;
        if (gamepad1.dpadDownWasPressed()) kV -= 0.01;
        if (gamepad1.dpadRightWasPressed()) kA += 0.001;
        if (gamepad1.dpadLeftWasPressed()) kA -= 0.001;
        if (gamepad1.rightBumperWasPressed()) kS += 0.01;
        if (gamepad1.leftBumperWasPressed()) kS -= 0.01;

        kV = MathUtils.clamp(kV, 0, 2.0);
        kA = MathUtils.clamp(kA, 0, 1.0);
        kS = MathUtils.clamp(kS, 0, 1.0);

        controller = new DriveMotorController(kV, kA, kS);


        switch (state) {
            case IDLE:
                if (aPressed) {
                    state = TestState.STATIC_TEST;
                    testTimer.reset();
                }
                targetVelocity = 0;
                targetAccel = 0;
                break;

            case STATIC_TEST:
                if (testTimer.seconds() < 3.0) {
                    targetVelocity = 1;
                    targetAccel = 0;
                } else {
                    state = TestState.VELOCITY_TEST;
                    velocityTestStep = 0;
                    testTimer.reset();
                }
                break;

            case VELOCITY_TEST:
                if (testTimer.seconds() > 2.0) {
                    if (velocityTestStep < testVelocities.length) {
                        targetVelocity = testVelocities[velocityTestStep];
                        targetAccel = 0;

                        if (testTimer.seconds() > 5.0) {
                            measuredPowers[velocityTestStep] = getCurrentPower();
                            velocityTestStep++;
                            testTimer.reset();
                        }
                    } else {
                        state = TestState.ACCEL_TEST;
                        testTimer.reset();
                    }
                }
                break;

            case ACCEL_TEST:
                if (testTimer.seconds() < 2.0) {
                    targetVelocity = testTimer.seconds();
                    targetAccel = 1.0;
                } else {
                    state = TestState.COMPLETE;
                    calculateResults();
                }
                break;

            case COMPLETE:
                if (bPressed) {
                    state = TestState.IDLE;
                    velocityTestStep = 0;
                }
                targetVelocity = 0;
                targetAccel = 0;
                break;
        }

        if (xPressed) {
            state = TestState.IDLE;
            targetVelocity = 0;
            targetAccel = 0;
        }

        double power = controller.calculate(
                targetVelocity / SwerveConstants.WHEEL_RADIUS,
                targetAccel / SwerveConstants.WHEEL_RADIUS,
                0,
                hardware.getBatteryVoltage()
        );

        hardware.setMotorPowers(power, power);

        displayTelemetry();
    }

    private double getCurrentPower() {
        return controller.calculate(
                targetVelocity / SwerveConstants.WHEEL_RADIUS,
                0,
                0,
                hardware.getBatteryVoltage()
        );
    }

    private void calculateResults() {
        double sumKV = 0;
        for (int i = 0; i < testVelocities.length; i++) {
            sumKV += measuredPowers[i] / (testVelocities[i] / SwerveConstants.WHEEL_RADIUS);
        }
        double calculatedKV = sumKV / testVelocities.length;

        telemetry.addData("Calculated kV", "%.4f", calculatedKV);
    }

    private void displayTelemetry() {
        telemetry.addData("State", state.toString());
        telemetry.addData("", "");

        telemetry.addData("Current Gains", "");
        telemetry.addData("  kV", "%.4f (D-pad Up/Down)", kV);
        telemetry.addData("  kA", "%.4f (D-pad Left/Right)", kA);
        telemetry.addData("  kS", "%.4f (Bumpers)", kS);
        telemetry.addData("", "");

        telemetry.addData("Target Velocity", "%.2f m/s", targetVelocity);
        telemetry.addData("Target Accel", "%.2f m/s²", targetAccel);
        telemetry.addData("", "");

        if (state == TestState.COMPLETE) {
            telemetry.addData("Results", "");
            calculateResults();
            telemetry.addData("", "");
            telemetry.addData("Copy to Constants", "");
            telemetry.addData("DRIVE_KV", "%.4f", kV);
            telemetry.addData("DRIVE_KA", "%.4f", kA);
            telemetry.addData("DRIVE_KS", "%.4f", kS);
            telemetry.addData("", "");
            telemetry.addData("Press B", "to reset and run again");
        } else if (state == TestState.IDLE) {
            telemetry.addData("Press A", "to start test");
            telemetry.addData("Press X", "to emergency stop");
        } else {
            telemetry.addData("Test Progress", "%.1f seconds", testTimer.seconds());
            telemetry.addData("Press X", "to emergency stop");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.setMotorPowers(0, 0);
    }
}
