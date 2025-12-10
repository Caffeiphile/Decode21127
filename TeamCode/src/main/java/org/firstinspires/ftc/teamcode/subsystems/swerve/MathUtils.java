package org.firstinspires.ftc.teamcode.subsystems.swerve;
public class MathUtils {
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double deadband(double value, double threshold) {
        return Math.abs(value) < threshold ? 0.0 : value;
    }

    public static double signedSquare(double value) {
        return value * Math.abs(value);
    }

    public static double lerp(double start, double end, double fraction) {
        return start + (end - start) * fraction;
    }

    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (2 * Math.PI);
        if (angle > Math.PI) angle -= 2 * Math.PI;
        else if (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static double getShortestAngularDistance(double fromAngle, double toAngle) {
        return normalizeAngle(toAngle - fromAngle);
    }

    public static double angleDifference(double angle1, double angle2) {
        return Math.abs(normalizeAngle(angle1 - angle2));
    }

    public static double interpolateAngles(double startAngle, double endAngle, double fraction) {
        double delta = getShortestAngularDistance(startAngle, endAngle);
        return normalizeAngle(startAngle + delta * fraction);
    }

    public static double wrapAngle2Pi(double angleRadians) {
        double angle = angleRadians % (2 * Math.PI);
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }
}