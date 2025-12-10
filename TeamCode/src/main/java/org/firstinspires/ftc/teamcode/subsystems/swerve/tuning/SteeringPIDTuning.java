package org.firstinspires.ftc.teamcode.subsystems.swerve.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@Config
@TeleOp(name="Steering PID Tuning", group="Tuning")
public class SteeringPIDTuning extends OpMode {
    private SwerveModuleHardware hardware;
    private SteeringPIDController controller;
    private ElapsedTime runtime;
    private ElapsedTime testTimer;

    static double kP = SwerveConstants.STEER_KP;
    static double kI = SwerveConstants.STEER_KI;
    static double kD = SwerveConstants.STEER_KD;

    private enum TestMode {
        MANUAL,
        STEP_RESPONSE,
        TRACKING,
        OSCILLATION
    }

    private TestMode mode = TestMode.MANUAL;
    private double targetHeading = 0;
    private double startHeading = 0;

    private double maxError = 0;
    private double settleTime = 0;
    private boolean hasSettled = false;
    private double overshoot = 0;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void init() {
        runtime = new ElapsedTime();
        testTimer = new ElapsedTime();

        hardware = new SwerveModuleHardware(
                hardwareMap,
                "cidTop",
                "cidBottom",
                "cidEncoder",
                true,
                false,
                0.0
        );

        controller = new SteeringPIDController(kP, kD, kI);

        telemetry.addData("Status", "Ready");
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

        if (gamepad1.dpadUpWasPressed()) kP += 0.1;
        if (gamepad1.dpadDownWasPressed()) kP = Math.max(0, kP - 0.1);
        if (gamepad1.dpadRightWasPressed()) kD += 0.01;
        if (gamepad1.dpadLeftWasPressed()) kD = Math.max(0, kD - 0.01);
        if (gamepad1.rightBumperWasPressed()) kI += 0.001;
        if (gamepad1.leftBumperWasPressed()) kI = Math.max(0, kI - 0.001);

        controller = new SteeringPIDController(kP, kD, kI);

        if (aPressed) {
            mode = TestMode.STEP_RESPONSE;
            startHeading = hardware.getModuleAngle();
            targetHeading = MathUtils.normalizeAngle(startHeading + Math.PI / 2);
            testTimer.reset();
            maxError = 0;
            hasSettled = false;
            overshoot = 0;
        }

        if (bPressed) {
            mode = TestMode.TRACKING;
            testTimer.reset();
        }

        if (xPressed) {
            mode = TestMode.OSCILLATION;
            testTimer.reset();
        }

        if (yPressed) {
            mode = TestMode.MANUAL;
            controller.reset();
        }

        double currentHeading = hardware.getModuleAngle();

        switch (mode) {
            case MANUAL:
                targetHeading = gamepad1.right_stick_x * Math.PI;
                break;

            case STEP_RESPONSE:
                double error = Math.abs(MathUtils.getShortestAngularDistance(currentHeading, targetHeading));
                maxError = Math.max(maxError, error);

                if (error < Math.toRadians(2) && !hasSettled) {
                    settleTime = testTimer.seconds();
                    hasSettled = true;
                }

                double errorSign = MathUtils.getShortestAngularDistance(targetHeading, currentHeading);
                if (errorSign < 0 && Math.abs(errorSign) > Math.toRadians(5)) {
                    overshoot = Math.max(overshoot, Math.abs(errorSign));
                }
                break;

            case TRACKING:
                targetHeading = Math.sin(testTimer.seconds() * 2) * Math.PI / 4;
                break;

            case OSCILLATION:
                if ((int)(testTimer.seconds() * 2) % 2 == 0) {
                    targetHeading = Math.PI / 4;
                } else {
                    targetHeading = -Math.PI / 4;
                }
                break;
        }

        double steeringOutput = controller.calculate(currentHeading, targetHeading);

        double topPower = SwerveConstants.STEER_GEAR_RATIO * steeringOutput;
        double bottomPower = SwerveConstants.STEER_GEAR_RATIO * steeringOutput;

        hardware.setMotorPowers(topPower, -bottomPower);

        displayTelemetry(currentHeading);
    }

    private void displayTelemetry(double currentHeading) {
        telemetry.addData("Mode", mode.toString());
        telemetry.addData("", "");

        telemetry.addData("PID Gains", "");
        telemetry.addData("  kP", "%.3f (D-pad Up/Down)", kP);
        telemetry.addData("  kD", "%.3f (D-pad Left/Right)", kD);
        telemetry.addData("  kI", "%.4f (Bumpers)", kI);
        telemetry.addData("", "");

        double error = MathUtils.getShortestAngularDistance(currentHeading, targetHeading);
        telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentHeading));
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        telemetry.addData("Error", "%.1f°", Math.toDegrees(error));
        telemetry.addData("", "");

        if (mode == TestMode.STEP_RESPONSE) {
            telemetry.addData("Step Response Results", "");
            telemetry.addData("  Max Error", "%.1f°", Math.toDegrees(maxError));
            if (hasSettled) {
                telemetry.addData("  Settle Time", "%.2f s", settleTime);
            } else {
                telemetry.addData("  Settle Time", "Not yet settled");
            }
            telemetry.addData("  Overshoot", "%.1f°", Math.toDegrees(overshoot));
            telemetry.addData("", "");

            String assessment = "";
            if (overshoot > Math.toRadians(10)) {
                assessment = "Too much overshoot - reduce kP or increase kD";
            } else if (!hasSettled && testTimer.seconds() > 2.0) {
                assessment = "Slow to settle - increase kP";
            } else if (hasSettled && settleTime < 0.5) {
                assessment = "Good response!";
            } else if (hasSettled) {
                assessment = "Could be faster - try increasing kP";
            }
            telemetry.addData("Assessment", assessment);
        }

        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("A", "90° Step Response Test");
        telemetry.addData("B", "Sine Wave Tracking Test");
        telemetry.addData("X", "Oscillation Test");
        telemetry.addData("Y", "Manual Control (Right Stick)");

        if (mode == TestMode.STEP_RESPONSE && hasSettled) {
            telemetry.addData("", "");
            telemetry.addData("Copy to Constants", "");
            telemetry.addData("STEER_KP", "%.3f", kP);
            telemetry.addData("STEER_KD", "%.3f", kD);
            telemetry.addData("STEER_KI", "%.4f", kI);
        }
        telemetry.addData("", "fixed?");
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.setMotorPowers(0, 0);
    }
}