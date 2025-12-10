package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class SteeringPIDController {
    private double kP;
    private double kD;
    private double kI;

    private double previousError = 0;
    private double accumulatedError = 0;
    private double previousTime = 0;

    private static final double MAX_INTEGRAL = 1.0;

    public SteeringPIDController(double kP, double kD, double kI) {
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
    }

    public SteeringPIDController(double kP, double kD) {
        this(kP, kD, 0.0);
    }

    public double calculate(double currentAngle, double targetAngle) {
        double currentTime = System.nanoTime() / 1E9;
        double error = MathUtils.getShortestAngularDistance(currentAngle, targetAngle);

        if (previousTime == 0) {
            previousTime = currentTime;
            previousError = error;
            return kP * error;
        }

        double dt = currentTime - previousTime;
        if (dt <= 0) dt = 0.001;

        double proportional = kP * error;

        double derivative = kD * (error - previousError) / dt;

        accumulatedError += error * dt;
        accumulatedError = MathUtils.clamp(accumulatedError, -MAX_INTEGRAL, MAX_INTEGRAL);
        double integral = kI * accumulatedError;

        previousError = error;
        previousTime = currentTime;

        return proportional + derivative + integral;
    }

    public void reset() {
        previousError = 0;
        accumulatedError = 0;
        previousTime = -1;
    }

    public void setKP(double kP) { this.kP = kP; }
    public void setKD(double kD) { this.kD = kD; }
    public void setKI(double kI) { this.kI = kI; }

    public double getKP() { return kP; }
    public double getKD() { return kD; }
    public double getKI() { return kI; }
}
