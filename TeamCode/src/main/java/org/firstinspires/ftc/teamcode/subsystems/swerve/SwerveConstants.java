package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class SwerveConstants {
    public static final double WHEEL_RADIUS = 0.048;
    public static final double DRIVE_GEAR_RATIO = 1.0;
    public static final double STEER_GEAR_RATIO = 1.3;
    public static final double MAX_WHEEL_SPEED = 2.0;

    public static final double DRIVE_KV = 0.1;
    public static final double DRIVE_KA = 0.00;
    public static final double DRIVE_KS = 0.02;
    public static final double BATTERY_NOMINAL_VOLTAGE = 12.0;

    public static final double STEER_KP = 0.800;
    public static final double STEER_KD = 0.0124;
    public static final double STEER_KI = 0.0;

    public static final double HEADING_CUTOFF =  Math.PI; // Math.toRadians(120);
    public static final double HEADING_DEADBAND = Math.toRadians(3);
    public static final boolean USE_SQUARED_COSINE = false;
    public static final double MAX_LOOP_TIME_MS = 20.0;

    public static final double WHEELBASE = 0.330;
    public static final double POD_OFFSET_FROM_CENTER = 0.120;

    public static final double MAX_TRANSLATION_SPEED = 2.0;
    public static final double MAX_ROTATION_SPEED = 2 * Math.PI;

    public static final double OMEGA_THRESHOLD = 0.1;
    public static final double VELOCITY_DEADBAND = 0.01;
    public static final double IDLE_THRESHOLD = 0.01;
    public static final double HOLDING_FACTOR = 0.3;

    public static final double PINPOINT_X_OFFSET = -95.0;
    public static final double PINPOINT_Y_OFFSET = 120.0;
}