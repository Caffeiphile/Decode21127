package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class SimpleSwerveDrive {

    private final SimpleSwerveModule[] modules;
    private final Vector2d[] modulePositions;

    public SimpleSwerveDrive(SimpleSwerveModule[] modules,
                             Vector2d[] modulePositions) {
        this.modules = modules;
        this.modulePositions = modulePositions;
    }

    public void drive(double vx, double vy, double omega) {

        // Limit chassis speed (FTC-safe)
        double maxSpeed = 1.0;
        double mag = Math.hypot(vx, vy);
        if (mag > maxSpeed) {
            vx *= maxSpeed / mag;
            vy *= maxSpeed / mag;
        }

        for (int i = 0; i < modules.length; i++) {

            Vector2d pos = modulePositions[i];

            double wx = vx - omega * -pos.y;
            double wy = vy + omega * pos.x;

            double speed = Math.hypot(wx, wy);
            double angle = Math.atan2(wy, wx);

            modules[i].set(speed, angle);
        }
    }
}
