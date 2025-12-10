package org.firstinspires.ftc.teamcode.subsystems.swerve;


public class FeedforwardController {
    private double kV;
    private double kA;
    private double kS;

    public FeedforwardController(double kV, double kA, double kS) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    public double calculate(double velocityRadPerSec, double accelRadPerSec2,
                            double currentBatteryVoltage) {
        double feedforward = kV * velocityRadPerSec + kA * accelRadPerSec2;

        if (Math.abs(velocityRadPerSec) < 0.1) {
            feedforward += kS * Math.tanh(velocityRadPerSec / 0.1);
        } else {
            feedforward += kS * Math.signum(velocityRadPerSec);
        }

        feedforward *= (12.0 / currentBatteryVoltage);

        return MathUtils.clamp(feedforward, -1.0, 1.0);
    }

    public double calculate(double velocityRadPerSec, double accelRadPerSec2) {
        return calculate(velocityRadPerSec, accelRadPerSec2, 12.0);
    }

    public void setKV(double kV) { this.kV = kV; }
    public void setKA(double kA) { this.kA = kA; }
    public void setKS(double kS) { this.kS = kS; }

    public double getKV() { return kV; }
    public double getKA() { return kA; }
    public double getKS() { return kS; }
}
