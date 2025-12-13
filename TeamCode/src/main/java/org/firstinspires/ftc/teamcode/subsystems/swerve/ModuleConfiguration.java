package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class ModuleConfiguration {
    public final String name;
    public final Vector2d position;
    public final String topMotorName;
    public final String bottomMotorName;
    public final String encoderName;
    public final boolean topMotorReversed;
    public final boolean bottomMotorReversed;
    public final double encoderOffset;

    public ModuleConfiguration(String name, Vector2d position,
                               String topMotorName, String bottomMotorName,
                               String encoderName,
                               boolean topMotorReversed, boolean bottomMotorReversed,
                               double encoderOffset) {
        this.name = name;
        this.position = position;
        this.topMotorName = topMotorName;
        this.bottomMotorName = bottomMotorName;
        this.encoderName = encoderName;
        this.topMotorReversed = topMotorReversed;
        this.bottomMotorReversed = bottomMotorReversed;
        this.encoderOffset = encoderOffset;
    }

    public static ModuleConfiguration createStandard(String name, Vector2d position) {
        return new ModuleConfiguration(
                name,
                position,
                name + "Top",
                name + "Bottom",
                name + "Encoder",
                true,
                false,
                0.0
        );
    }

    public static ModuleConfiguration createStandard(String name, Vector2d position, double encoderOffset) {
        return new ModuleConfiguration(
                name,
                position,
                name + "Top",
                name + "Bottom",
                name + "Encoder",
                true,
                false,
                encoderOffset
        );
    }

    @Override
    public String toString() {
        return String.format("%s @ %s", name, position);
    }
}

