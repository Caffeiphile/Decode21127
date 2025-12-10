package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class Vector2d {
    public final double x;
    public final double y;
    private Double cachedMagnitude = null;

    public static final Vector2d ZERO = new Vector2d(0, 0);

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2d fromPolar(double magnitude, double angleRadians) {
        return new Vector2d(
                magnitude * Math.cos(angleRadians),
                magnitude * Math.sin(angleRadians)
        );
    }

    public double getMagnitude() {
        if (cachedMagnitude == null) {
            cachedMagnitude = Math.hypot(x, y);
        }
        return cachedMagnitude;
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public Vector2d rotated(double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        return new Vector2d(
                x * cos - y * sin,
                x * sin + y * cos
        );
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(x + other.x, y + other.y);
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(x - other.x, y - other.y);
    }

    public Vector2d times(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }

    public double dot(Vector2d other) {
        return x * other.x + y * other.y;
    }

    public Vector2d normalized() {
        double mag = getMagnitude();
        return mag == 0 ? ZERO : new Vector2d(x / mag, y / mag);
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }
}

