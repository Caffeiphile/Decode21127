package org.firstinspires.ftc.teamcode.subsystems.swerve.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

import java.util.ArrayList;

@TeleOp(name="Ziegler-Nichols Auto Tuning", group="Tuning")
public class ZieglerNicholsAutoTuning extends OpMode {

    private SwerveModuleHardware hardware;
    private SteeringPIDController controller;
    private ElapsedTime runtime;

    private enum TuningPhase {
        SETUP,
        FINDING_KU,
        MEASURING_OSCILLATION,
        CALCULATING,
        COMPLETE
    }

    private TuningPhase phase = TuningPhase.SETUP;

    private double testKp = 0.1;
    private double kU = 0;
    private double tU = 0;

    private double targetHeading = Math.PI / 4;
    private ArrayList<Double> errorHistory = new ArrayList<>();
    private ArrayList<Double> timeHistory = new ArrayList<>();

    private ArrayList<Double> peakTimes = new ArrayList<>();
    private ArrayList<Double> peakErrors = new ArrayList<>();

    private int oscillationCount = 0;
    private double lastError = 0;
    private double lastErrorSign = 0;
    private boolean detectingPeaks = false;

    private double calculatedKp = 0;
    private double calculatedKi = 0;
    private double calculatedKd = 0;

    private int tuningProfile = 0;
    private String[] profileNames = {
            "PID (Standard)",
            "PID (Some Overshoot)",
            "PID (No Overshoot)",
            "PD Only",
            "PI Only",
            "P Only"
    };

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private static final int MIN_OSCILLATIONS = 3;
    private static final double OSCILLATION_THRESHOLD = 0.02;
    private static final double STABILITY_THRESHOLD = 0.15;

    @Override
    public void init() {
        runtime = new ElapsedTime();

        hardware = new SwerveModuleHardware(
                hardwareMap,
                "cidTop",
                "cidBottom",
                "cidEncoder",
                true,
                true,
                0.0
        );

        controller = new SteeringPIDController(0, 0, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Press A to begin auto-tuning");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        boolean xPressed = gamepad1.x && !lastX;
        boolean yPressed = gamepad1.y && !lastY;
        boolean dpadUpPressed = gamepad1.dpad_up && !lastDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDown;

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        double currentHeading = hardware.getModuleAngle();
        double error = MathUtils.getShortestAngularDistance(currentHeading, targetHeading);

        switch (phase) {
            case SETUP:
                hardware.setMotorPowers(0, 0);

                if (dpadUpPressed) {
                    tuningProfile = (tuningProfile + 1) % profileNames.length;
                }
                if (dpadDownPressed) {
                    tuningProfile = (tuningProfile - 1 + profileNames.length) % profileNames.length;
                }

                if (aPressed) {
                    phase = TuningPhase.FINDING_KU;
                    testKp = 0.1;
                    errorHistory.clear();
                    timeHistory.clear();
                    runtime.reset();
                    controller = new SteeringPIDController(testKp, 0, 0);
                }
                break;

            case FINDING_KU:
                errorHistory.add(error);
                timeHistory.add(runtime.seconds());

                if (errorHistory.size() > 100) {
                    errorHistory.remove(0);
                    timeHistory.remove(0);
                }

                if (isOscillating()) {
                    kU = testKp;
                    phase = TuningPhase.MEASURING_OSCILLATION;
                    peakTimes.clear();
                    peakErrors.clear();
                    oscillationCount = 0;
                    detectingPeaks = true;
                    lastErrorSign = Math.signum(error);
                    runtime.reset();
                } else if (runtime.seconds() > 2.0) {
                    testKp += 0.1;
                    targetHeading += Math.PI / 4;
                    controller = new SteeringPIDController(testKp, 0, 0);
                    errorHistory.clear();
                    timeHistory.clear();
                    runtime.reset();

                    if (testKp > 20.0) {
                        telemetry.addData("Error", "Could not find Ku");
                        phase = TuningPhase.SETUP;
                    }
                }
                break;

            case MEASURING_OSCILLATION:
                double errorSign = Math.signum(error);

                if (errorSign != lastErrorSign && Math.abs(error) > OSCILLATION_THRESHOLD) {
                    peakTimes.add(runtime.seconds());
                    peakErrors.add(Math.abs(lastError));
                    oscillationCount++;
                }

                lastErrorSign = errorSign;
                lastError = error;

                if (oscillationCount >= MIN_OSCILLATIONS * 2) {
                    phase = TuningPhase.CALCULATING;
                    calculatePeriod();
                    calculateGains();
                }

                if (runtime.seconds() > 15.0 && oscillationCount < 2) {
                    telemetry.addData("Error", "Oscillations not stable");
                    phase = TuningPhase.SETUP;
                }
                break;

            case CALCULATING:
                hardware.setMotorPowers(0, 0);
                phase = TuningPhase.COMPLETE;
                break;

            case COMPLETE:
                hardware.setMotorPowers(0, 0);

                if (bPressed) {
                    targetHeading += Math.PI / 4;
                    controller = new SteeringPIDController(calculatedKp, calculatedKd, calculatedKi);
                    testCalculatedGains(currentHeading, error);
                }

                if (xPressed) {
                    phase = TuningPhase.SETUP;
                    testKp = 0.1;
                    errorHistory.clear();
                    timeHistory.clear();
                    peakTimes.clear();
                    peakErrors.clear();
                }
                break;
        }

        if (phase == TuningPhase.FINDING_KU || phase == TuningPhase.MEASURING_OSCILLATION) {
            double steeringOutput = controller.calculate(currentHeading, targetHeading);
            double topPower = SwerveConstants.STEER_GEAR_RATIO * steeringOutput;
            double bottomPower = SwerveConstants.STEER_GEAR_RATIO * steeringOutput;
            hardware.setMotorPowers(topPower, bottomPower);
        }

        if (yPressed) {
            hardware.setMotorPowers(0, 0);
            phase = TuningPhase.SETUP;
        }

        displayTelemetry(currentHeading, error);
    }

    private boolean isOscillating() {
        if (errorHistory.size() < 50) return false;

        int crossings = 0;
        double lastSign = Math.signum(errorHistory.get(0));

        for (int i = 1; i < errorHistory.size(); i++) {
            double currentSign = Math.signum(errorHistory.get(i));
            if (currentSign != lastSign && Math.abs(errorHistory.get(i)) > OSCILLATION_THRESHOLD) {
                crossings++;
            }
            lastSign = currentSign;
        }

        if (crossings < 4) return false;

        double maxError = 0;
        for (double err : errorHistory) {
            maxError = Math.max(maxError, Math.abs(err));
        }

        double recentMax = 0;
        for (int i = errorHistory.size() - 20; i < errorHistory.size(); i++) {
            recentMax = Math.max(recentMax, Math.abs(errorHistory.get(i)));
        }

        return Math.abs(maxError - recentMax) < STABILITY_THRESHOLD;
    }

    private void calculatePeriod() {
        if (peakTimes.size() < 2) {
            tU = 0;
            return;
        }

        ArrayList<Double> periods = new ArrayList<>();
        for (int i = 1; i < peakTimes.size(); i++) {
            double period = (peakTimes.get(i) - peakTimes.get(i - 1)) * 2;
            periods.add(period);
        }

        double sum = 0;
        for (double period : periods) {
            sum += period;
        }
        tU = sum / periods.size();
    }

    private void calculateGains() {
        switch (tuningProfile) {
            case 0:
                calculatedKp = 0.6 * kU;
                calculatedKi = 1.2 * kU / tU;
                calculatedKd = 0.075 * kU * tU;
                break;
            case 1:
                calculatedKp = 0.33 * kU;
                calculatedKi = 0.66 * kU / tU;
                calculatedKd = 0.11 * kU * tU;
                break;
            case 2:
                calculatedKp = 0.2 * kU;
                calculatedKi = 0.40 * kU / tU;
                calculatedKd = 0.066 * kU * tU;
                break;
            case 3:
                calculatedKp = 0.8 * kU;
                calculatedKi = 0;
                calculatedKd = 0.1 * kU * tU;
                break;
            case 4:
                calculatedKp = 0.45 * kU;
                calculatedKi = 0.54 * kU / tU;
                calculatedKd = 0;
                break;
            case 5:
                calculatedKp = 0.5 * kU;
                calculatedKi = 0;
                calculatedKd = 0;
                break;
        }
    }

    private void testCalculatedGains(double currentHeading, double error) {
        double steeringOutput = controller.calculate(currentHeading, targetHeading);
        double topPower = SwerveConstants.STEER_GEAR_RATIO * steeringOutput;
        double bottomPower = SwerveConstants.STEER_GEAR_RATIO * steeringOutput;
        hardware.setMotorPowers(topPower, bottomPower);
    }

    private void displayTelemetry(double currentHeading, double error) {
        telemetry.addData("Phase", phase.toString());
        telemetry.addData("", "");

        switch (phase) {
            case SETUP:
                telemetry.addData("Tuning Profile", profileNames[tuningProfile]);
                telemetry.addData("", "D-pad Up/Down to change profile");
                telemetry.addData("", "");
                telemetry.addData("Instructions", "");
                telemetry.addData("1", "Select desired profile");
                telemetry.addData("2", "Ensure module can rotate freely");
                telemetry.addData("3", "Press A to start auto-tuning");
                telemetry.addData("", "");
                telemetry.addData("Profile Descriptions", "");
                telemetry.addData("Standard", "Balanced response");
                telemetry.addData("Some Overshoot", "Faster, slight overshoot");
                telemetry.addData("No Overshoot", "Conservative, no overshoot");
                break;

            case FINDING_KU:
                telemetry.addData("Status", "Finding critical gain Ku...");
                telemetry.addData("Test Kp", "%.3f", testKp);
                telemetry.addData("Time", "%.1f s", runtime.seconds());
                telemetry.addData("", "");
                telemetry.addData("Current Error", "%.1f°", Math.toDegrees(error));
                telemetry.addData("Target", "%.1f°", Math.toDegrees(targetHeading));
                telemetry.addData("Current", "%.1f°", Math.toDegrees(currentHeading));
                telemetry.addData("", "");
                telemetry.addData("Looking for", "Steady oscillation");
                break;

            case MEASURING_OSCILLATION:
                telemetry.addData("Status", "Measuring oscillation period...");
                telemetry.addData("Ku (Critical Gain)", "%.3f", kU);
                telemetry.addData("Oscillations Detected", "%d / %d",
                        oscillationCount / 2, MIN_OSCILLATIONS);
                telemetry.addData("Time", "%.1f s", runtime.seconds());
                telemetry.addData("", "");
                telemetry.addData("Current Error", "%.1f°", Math.toDegrees(error));
                break;

            case COMPLETE:
                telemetry.addData("Status", "✓ Auto-Tuning Complete!");
                telemetry.addData("", "");
                telemetry.addData("Profile Used", profileNames[tuningProfile]);
                telemetry.addData("", "");
                telemetry.addData("Measured Values", "");
                telemetry.addData("  Ku (Critical Gain)", "%.3f", kU);
                telemetry.addData("  Tu (Period)", "%.3f s", tU);
                telemetry.addData("  Oscillations", "%d", oscillationCount / 2);
                telemetry.addData("", "");
                telemetry.addData("Calculated Gains", "");
                telemetry.addData("  Kp", "%.4f", calculatedKp);
                telemetry.addData("  Ki", "%.4f", calculatedKi);
                telemetry.addData("  Kd", "%.4f", calculatedKd);
                telemetry.addData("", "");
                telemetry.addData("Copy to SwerveConstants.java", "");
                telemetry.addData("STEER_KP", "%.4f", calculatedKp);
                telemetry.addData("STEER_KI", "%.4f", calculatedKi);
                telemetry.addData("STEER_KD", "%.4f", calculatedKd);
                telemetry.addData("", "");
                telemetry.addData("Controls", "");
                telemetry.addData("B", "Test calculated gains");
                telemetry.addData("X", "Run tuning again");
                telemetry.addData("Y", "Emergency stop");
                break;
        }

        if (phase != TuningPhase.COMPLETE && phase != TuningPhase.SETUP) {
            telemetry.addData("", "");
            telemetry.addData("Press Y", "Emergency stop");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.setMotorPowers(0, 0);
    }
}
