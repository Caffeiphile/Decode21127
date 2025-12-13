package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Test 2: Kinematics", group="Swerve Test")
public class Test2_Kinematics extends OpMode {
    private SwerveKinematics kinematics;
    private double speedMultiplier = 0.7;

    @Override
    public void init() {
        double offset = SwerveConstants.POD_OFFSET_FROM_CENTER;
        kinematics = new SwerveKinematics(new Vector2d[] {
                new Vector2d(-offset, offset),
                new Vector2d(offset, -offset)
        });
        telemetry.addData("Test", "Kinematics Calculation");
        telemetry.update();
    }

    @Override
    public void loop() {
        speedMultiplier = gamepad1.left_bumper ? 0.3 : (gamepad1.right_bumper ? 1.0 : 0.7);

        double vx = MathUtils.deadband(-gamepad1.left_stick_y, 0.05) * SwerveConstants.MAX_TRANSLATION_SPEED * speedMultiplier;
        double vy = MathUtils.deadband(gamepad1.left_stick_x, 0.05) * SwerveConstants.MAX_TRANSLATION_SPEED * speedMultiplier;
        double omega = MathUtils.deadband(gamepad1.right_stick_x, 0.05) * SwerveConstants.MAX_ROTATION_SPEED * speedMultiplier;

        SwerveKinematics.ModuleState[] states = kinematics.calculateModuleStates(vx, vy, omega);
        kinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_WHEEL_SPEED);

        telemetry.addData("Robot Velocities", "");
        telemetry.addData("  vx", "%.3f m/s", vx);
        telemetry.addData("  vy", "%.3f m/s", vy);
        telemetry.addData("  omega", "%.3f rad/s", omega);
        telemetry.addData("", "");
        telemetry.addData("Module States", "");
        telemetry.addData("Left", "vmx=%.3f vmy=%.3f", states[0].velocityVector.x, states[0].velocityVector.y);
        telemetry.addData("Right", "vmx=%.3f vmy=%.3f", states[1].velocityVector.x, states[1].velocityVector.y);
        telemetry.addData("", "");
        telemetry.addData("✓ PASS if", "forward: both vmx same, zero vmy");
        telemetry.addData("✓ PASS if", "rotate: vmx opposite signs");
        telemetry.update();
    }
}
