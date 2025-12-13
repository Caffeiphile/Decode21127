package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Test 1: Gamepad Input", group="Swerve Test")
public class Test1_GamepadInput extends OpMode {
    private double speedMultiplier = 0.7;

    @Override
    public void init() {
        telemetry.addData("Test", "Gamepad Input Processing");
        telemetry.addData("Instructions", "Move sticks and press buttons");
        telemetry.update();
    }

    @Override
    public void loop() {
        speedMultiplier = gamepad1.left_bumper ? 0.3 : (gamepad1.right_bumper ? 1.0 : 0.7);

        double rawVx = -gamepad1.left_stick_y;
        double rawVy = gamepad1.left_stick_x;
        double rawOmega = gamepad1.right_stick_x;

        double vx = MathUtils.deadband(rawVx, 0.05) * SwerveConstants.MAX_TRANSLATION_SPEED * speedMultiplier;
        double vy = MathUtils.deadband(rawVy, 0.05) * SwerveConstants.MAX_TRANSLATION_SPEED * speedMultiplier;
        double omega = MathUtils.deadband(rawOmega, 0.05) * SwerveConstants.MAX_ROTATION_SPEED * speedMultiplier;

        telemetry.addData("Raw Inputs", "");
        telemetry.addData("  Left Stick Y", "%.3f", rawVx);
        telemetry.addData("  Left Stick X", "%.3f", rawVy);
        telemetry.addData("  Right Stick X", "%.3f", rawOmega);
        telemetry.addData("", "");
        telemetry.addData("After Deadband & Scaling", "");
        telemetry.addData("  vx", "%.3f m/s", vx);
        telemetry.addData("  vy", "%.3f m/s", vy);
        telemetry.addData("  omega", "%.3f rad/s", omega);
        telemetry.addData("", "");
        telemetry.addData("Speed Multiplier", "%.1f (%s)", speedMultiplier,
                speedMultiplier == 0.3 ? "PRECISION" : (speedMultiplier == 1.0 ? "TURBO" : "NORMAL"));
        telemetry.addData("", "");
        telemetry.addData("✓ PASS if", "values respond to stick movement");
        telemetry.update();
    }
}
