package org.firstinspires.ftc.teamcode.subsystems.swerve;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

public class DriveMotorController {
    private final ControlSystem controller;

    public DriveMotorController(double kV, double kA, double kS) {
        this.controller = ControlSystem.builder()
                .basicFF(kV, kA, kS)
                .build();
    }

    public double calculate(double targetVelocity, double targetAccel, double currentVelocity, double batteryVoltage) {
        controller.setGoal(new KineticState(0, targetVelocity, targetAccel));
        double feedforward = controller.calculate(new KineticState(0, currentVelocity));
        feedforward *= (12.0 / batteryVoltage);
        return MathUtils.clamp(feedforward, -1.0, 1.0);
    }

    public double calculate(double targetVelocity, double targetAccel, double currentVelocity) {
        return calculate(targetVelocity, targetAccel, currentVelocity, 12.0);
    }
}
