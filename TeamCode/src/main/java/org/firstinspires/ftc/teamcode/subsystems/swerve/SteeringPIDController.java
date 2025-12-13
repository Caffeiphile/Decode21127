package org.firstinspires.ftc.teamcode.subsystems.swerve;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.control.KineticState;

public class SteeringPIDController {
    private final ControlSystem controller;

    public SteeringPIDController(double kP, double kD, double kI) {
        this.controller = ControlSystem.builder()
                .posPid(kP, kI, kD)
                .build();
    }

    public SteeringPIDController(double kP, double kD) {
        this(kP, kD, 0.0);
    }

    public double calculate(double currentAngle, double targetAngle) {
        double error = MathUtils.getShortestAngularDistance(currentAngle, targetAngle);
        controller.setGoal(new KineticState(0.0, 0.0));
        return controller.calculate(new KineticState(error, 0.0));
    }

    public void reset() {
        controller.reset();
    }
}
