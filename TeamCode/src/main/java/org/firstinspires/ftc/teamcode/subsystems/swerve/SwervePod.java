package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class SwervePod {
    private final SwerveModuleHardware hardware;
    private final Vector2d position;
    private final FeedforwardController driveFF;
    private final SteeringPIDController steeringPID;

    private Vector2d forwardVector;
    private Vector2d sideVector;
    private Vector2d turnVector;

    private double currentDriveVelocity = 0;
    private double currentDriveAcceleration = 0;
    private double currentTargetHeading = 0;
    private double heldHeading = 0;
    private boolean isIdle = false;
    private boolean isFlipped = false;

    public SwervePod(SwerveModuleHardware hardware, Vector2d position,
                     FeedforwardController driveFF, SteeringPIDController steeringPID) {
        this.hardware = hardware;
        this.position = position;
        this.driveFF = driveFF;
        this.steeringPID = steeringPID;

        this.forwardVector = new Vector2d(1, 0);
        this.sideVector = new Vector2d(0, 1);
        this.turnVector = new Vector2d(-position.y, position.x).normalized();
    }

    public void setUnitVectors(Vector2d forward, Vector2d side, Vector2d turn) {
        this.forwardVector = forward.normalized();
        this.sideVector = side.normalized();
        this.turnVector = turn.normalized();
    }

    public void updateModule(double forwardScalar, double sideScalar, double turnScalar) {
        Vector2d velocityVector = forwardVector.times(forwardScalar)
                .plus(sideVector.times(sideScalar))
                .plus(turnVector.times(turnScalar));

        double driveVel = velocityVector.getMagnitude();
        double targetHeading = velocityVector.getAngle();
        double driveAccel = 0;

        double currentHeading = hardware.getModuleAngle();

        double headingError = MathUtils.getShortestAngularDistance(currentHeading, targetHeading);

        if (Math.abs(headingError) > Math.PI / 2) {
            targetHeading = MathUtils.normalizeAngle(targetHeading + Math.PI);
            headingError = MathUtils.getShortestAngularDistance(currentHeading, targetHeading);
            driveVel *= -1;
            driveAccel *= -1;
        }

        if (Math.abs(driveVel) < SwerveConstants.IDLE_THRESHOLD) {
            if (!isIdle) {
                heldHeading = currentHeading;
                isIdle = true;
            }
            targetHeading = heldHeading;
        } else {
            isIdle = false;
            heldHeading = targetHeading;
        }

        double compensation = calculateCosineCompensation(headingError);
        driveVel *= compensation;
        driveAccel *= compensation;

        double steeringVelocity = steeringPID.calculate(currentHeading, targetHeading);

        if (isIdle) {
            steeringVelocity *= SwerveConstants.HOLDING_FACTOR;
        }

        double driveMotorVel = driveVel / SwerveConstants.WHEEL_RADIUS;
        double steerMotorVel = steeringVelocity;

        double topMotorVel = SwerveConstants.DRIVE_GEAR_RATIO * driveMotorVel +
                SwerveConstants.STEER_GEAR_RATIO * steerMotorVel;
        double bottomMotorVel = SwerveConstants.DRIVE_GEAR_RATIO * driveMotorVel -
                SwerveConstants.STEER_GEAR_RATIO * steerMotorVel;

        double topMotorAccel = SwerveConstants.DRIVE_GEAR_RATIO * (driveAccel / SwerveConstants.WHEEL_RADIUS);
        double bottomMotorAccel = topMotorAccel;

        double batteryVoltage = hardware.getBatteryVoltage();
        double topPower = driveFF.calculate(topMotorVel, topMotorAccel, batteryVoltage);
        double bottomPower = driveFF.calculate(bottomMotorVel, bottomMotorAccel, batteryVoltage);

        hardware.setMotorPowers(topPower, bottomPower);

        this.currentDriveVelocity = driveVel;
        this.currentDriveAcceleration = driveAccel;
        this.currentTargetHeading = targetHeading;
    }

    private double calculateCosineCompensation(double headingError) {
        double absError = Math.abs(headingError);

        if (absError > SwerveConstants.HEADING_CUTOFF) {
            return 0.0;
        }

        if (absError < SwerveConstants.HEADING_DEADBAND) {
            return 1.0;
        }

        double cosineFactor = Math.cos(headingError);

        if (SwerveConstants.USE_SQUARED_COSINE) {
            return cosineFactor * Math.abs(cosineFactor);
        } else {
            return cosineFactor;
        }
    }

    public ModuleState getModuleState() {
        return new ModuleState(
                hardware.getModuleAngle(),
                currentTargetHeading,
                currentDriveVelocity,
                currentDriveAcceleration
        );
    }

    public void reset() {
        steeringPID.reset();
        isIdle = false;
        isFlipped = false;
    }

    public Vector2d getPosition() {
        return position;
    }

    public static class ModuleState {
        public final double currentHeading;
        public final double targetHeading;
        public final double velocity;
        public final double acceleration;

        public ModuleState(double currentHeading, double targetHeading,
                           double velocity, double acceleration) {
            this.currentHeading = currentHeading;
            this.targetHeading = targetHeading;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        public double getHeadingError() {
            return MathUtils.getShortestAngularDistance(currentHeading, targetHeading);
        }
    }
}