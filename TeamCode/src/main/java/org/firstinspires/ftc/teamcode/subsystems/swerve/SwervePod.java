package org.firstinspires.ftc.teamcode.subsystems.swerve;


public class SwervePod {
    private final SwerveModuleHardware hardware;
    private final Vector2d position;
    private final DriveMotorController driveController;
    private final SteeringPIDController steeringController;

    private Vector2d forwardVector;
    private Vector2d sideVector;
    private Vector2d turnVector;

    private double currentDriveVelocity = 0;
    private double currentTargetHeading = 0;
    private double heldHeading = 0;
    private boolean isIdle = false;

    public SwervePod(SwerveModuleHardware hardware, Vector2d position,
                     DriveMotorController driveController, SteeringPIDController steeringController) {
        this.hardware = hardware;
        this.position = position;
        this.driveController = driveController;
        this.steeringController = steeringController;

        this.forwardVector = new Vector2d(1, 0);
        this.sideVector = new Vector2d(0, 1);
        this.turnVector = new Vector2d(position.x,-position.y).normalized();
    }

    public void updateModule(double driveVelocity, double targetHeading) {

//        double currentHeading = hardware.getModuleAngle();
//
//        // Shortest-path steering
//        double headingError =
//                MathUtils.getShortestAngularDistance(currentHeading, targetHeading);
//
//        // Flip wheel direction if > 90° away
//        if (Math.abs(headingError) > Math.PI / 2) {
//            targetHeading = MathUtils.normalizeAngle(targetHeading + Math.PI);
//            headingError =
//                    MathUtils.getShortestAngularDistance(currentHeading, targetHeading);
//            driveVelocity *= -1.0;
//        }
//
//        // Idle heading hold
//        if (Math.abs(driveVelocity) < SwerveConstants.IDLE_THRESHOLD) {
//            targetHeading = currentHeading;
//            driveVelocity = 0.0;
//        }
//
//        // Steering PID
//        double steeringVelocity =
//                steeringController.calculate(currentHeading, targetHeading);
//
//        // Convert wheel linear velocity → motor velocity
//        double driveMotorVel =
//                driveVelocity / SwerveConstants.WHEEL_RADIUS;
//
//        // Differential swerve math (NOW CORRECT)
//        double topMotorVel =
//                SwerveConstants.DRIVE_GEAR_RATIO * driveMotorVel
//                        + SwerveConstants.STEER_GEAR_RATIO * steeringVelocity;
//
//        double bottomMotorVel =
//                SwerveConstants.DRIVE_GEAR_RATIO * driveMotorVel
//                        - SwerveConstants.STEER_GEAR_RATIO * steeringVelocity;
//
//        double batteryVoltage = hardware.getBatteryVoltage();
//
//        double topPower =
//                driveController.calculate(topMotorVel, 0, batteryVoltage);
//        double bottomPower =
//                driveController.calculate(bottomMotorVel, 0, batteryVoltage);
//
//        // HARD CLAMP — prevents brownouts
//        topPower = Math.max(-1.0, Math.min(1.0, topPower));
//        bottomPower = Math.max(-1.0, Math.min(1.0, bottomPower));
//
//        hardware.setMotorPowers(topPower, bottomPower);
//
//        this.currentDriveVelocity = driveVelocity;
//        this.currentTargetHeading = targetHeading;

        double currentHeading = hardware.getModuleAngle();

        double headingError =
                MathUtils.getShortestAngularDistance(currentHeading, targetHeading);

        if (Math.abs(headingError) > Math.PI / 2) {
            targetHeading = MathUtils.normalizeAngle(targetHeading + Math.PI);
            driveVelocity *= -1;
        }

        double steeringVelocity =
                steeringController.calculate(currentHeading, targetHeading);

        double driveMotorVel =
                driveVelocity / SwerveConstants.WHEEL_RADIUS;
        if (headingError>Math.toRadians(60)){
            driveMotorVel = 0;
        }

        double topMotorVel =
                SwerveConstants.DRIVE_GEAR_RATIO * driveMotorVel
                        + SwerveConstants.STEER_GEAR_RATIO * steeringVelocity;

        double bottomMotorVel =
                SwerveConstants.DRIVE_GEAR_RATIO * driveMotorVel
                        - SwerveConstants.STEER_GEAR_RATIO * steeringVelocity;

        double batteryVoltage = hardware.getBatteryVoltage();

        double topPower =
                driveController.calculate(topMotorVel, 0, batteryVoltage);
        double bottomPower =
                driveController.calculate(bottomMotorVel, 0, batteryVoltage);

        topPower = Math.max(-1.0, Math.min(1.0, topPower));
        bottomPower = Math.max(-1.0, Math.min(1.0, bottomPower));

        hardware.setMotorPowers(topPower, bottomPower);
    }


    private double calculateCosineCompensation(double headingError) {
        double absError = Math.abs(headingError);
        if (absError > SwerveConstants.HEADING_CUTOFF) return 0.0;
        if (absError < SwerveConstants.HEADING_DEADBAND) return 1.0;

        double cosineFactor = Math.cos(headingError);
        return SwerveConstants.USE_SQUARED_COSINE ? cosineFactor * Math.abs(cosineFactor) : cosineFactor;
    }

    public ModuleState getModuleState() {
        return new ModuleState(hardware.getModuleAngle(), currentTargetHeading, currentDriveVelocity,
                hardware.getTopMotorPower(), hardware.getBottomMotorPower());
    }

    public static class ModuleState {
        public final double currentHeading;
        public final double targetHeading;
        public final double velocity;
        public final double topMotorPower;
        public final double bottomMotorPower;

        public ModuleState(double currentHeading, double targetHeading, double velocity,
                           double topMotorPower, double bottomMotorPower) {
            this.currentHeading = currentHeading;
            this.targetHeading = targetHeading;
            this.velocity = velocity;
            this.topMotorPower = topMotorPower;
            this.bottomMotorPower = bottomMotorPower;
        }

        public double getHeadingError() {
            return MathUtils.getShortestAngularDistance(currentHeading, targetHeading);
        }
    }
}
