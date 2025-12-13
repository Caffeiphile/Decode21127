package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class SimpleSwerveModule {

    private final SwerveModuleHardware hardware;
    private final SteeringPIDController steerPID;

    public double podSpeed = 0;
    public double podError = 0;

    public SimpleSwerveModule(SwerveModuleHardware hardware,
                              SteeringPIDController steerPID) {
        this.hardware = hardware;
        this.steerPID = steerPID;
    }

    public void set(double speed, double targetAngle) {

        double currentAngle = hardware.getModuleAngle();
        double error =
                MathUtils.getShortestAngularDistance(currentAngle, targetAngle);
        double finalTarget = targetAngle;

        // Flip wheel direction if shorter
        if (Math.abs(error) > Math.PI / 2) {
            speed = Math.abs(speed) * -1.0;
            finalTarget = MathUtils.normalizeAngle(targetAngle + Math.PI);
            error = MathUtils.getShortestAngularDistance(currentAngle, finalTarget);
        }

        // --- STEERING CONTROL ---
        double steerOutput = steerPID.calculate(currentAngle, finalTarget);

        double steerPower = clamp(steerOutput, 0.5);

        // --- STEERING AUTHORITY GATING ---
        double absError = Math.abs(error);

        double steerScale;
        if (absError > Math.toRadians(20)) {
            // Far away → full steering
            steerScale = 1.0;
        } else if (absError > Math.toRadians(5)) {
            // Transition region
            steerScale = 0.7;
        } else {
            // Close enough → almost no steering
            steerScale = 0.2;
        }

        steerPower *= steerScale;

        // --- DRIVE / STEER MIX ---
        double drivePower = clamp(speed, 1);

        podSpeed = drivePower;
        podError = error;

        double topPower = drivePower + steerPower;
        double bottomPower = drivePower - steerPower;

        topPower = clamp(topPower, 1.0);
        bottomPower = clamp(bottomPower, 1.0);

        hardware.setMotorPowers(topPower, bottomPower);
    }


    private double clamp(double val, double max) {
        return Math.max(-max, Math.min(max, val));
    }
}
