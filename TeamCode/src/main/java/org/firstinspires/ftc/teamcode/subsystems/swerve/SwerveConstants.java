package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class SwerveConstants {
    public static final double WHEEL_RADIUS = 0.048;
    public static final double DRIVE_GEAR_RATIO = 1.0;
    public static final double STEER_GEAR_RATIO = 0.3;
    public static final double MAX_WHEEL_SPEED = 2.0;

    public static final double DRIVE_KV = 0.5;
    public static final double DRIVE_KA = 0.05;
    public static final double DRIVE_KS = 0.1;
    public static final double BATTERY_NOMINAL_VOLTAGE = 12.0;

    public static final double STEER_KP = 2.0;
    public static final double STEER_KD = 0.1;
    public static final double STEER_KI = 0.0;

    public static final double HEADING_CUTOFF = Math.toRadians(120);
    public static final double HEADING_DEADBAND = Math.toRadians(3);
    public static final boolean USE_SQUARED_COSINE = true;
    public static final double MAX_LOOP_TIME_MS = 20.0;

    public static final double WHEELBASE = 0.254;
    public static final double TRACK_WIDTH = 0.254;

    public static final double MAX_TRANSLATION_SPEED = 2.0;
    public static final double MAX_ROTATION_SPEED = 2 * Math.PI;

    public static final double OMEGA_THRESHOLD = 0.1;
    public static final double VELOCITY_DEADBAND = 0.01;
    public static final double IDLE_THRESHOLD = 0.01;
    public static final double HOLDING_FACTOR = 0.3;
}