package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Test 3: SwervePod Input", group="Swerve Test")
public class Test3_SwervePodInput extends OpMode {
    private SwervePod pod;

    @Override
    public void init() {
        SwerveModuleHardware hardware = new SwerveModuleHardware(hardwareMap, "cidTop", "cidBottom",
                "cidEncoder", true, true, -3.0978);
        DriveMotorController driveController = new DriveMotorController(
                SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA, SwerveConstants.DRIVE_KS);
        SteeringPIDController steeringController = new SteeringPIDController(
                SwerveConstants.STEER_KP, SwerveConstants.STEER_KD, SwerveConstants.STEER_KI);

        double offset = SwerveConstants.POD_OFFSET_FROM_CENTER;
        pod = new SwervePod(hardware, new Vector2d(-offset, offset), driveController, steeringController);

        telemetry.addData("Test", "SwervePod receives correct inputs");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forwardScalar = MathUtils.deadband(-gamepad1.left_stick_y, 0.05);
        double sideScalar = MathUtils.deadband(gamepad1.left_stick_x, 0.05);
        double turnScalar = MathUtils.deadband(gamepad1.right_stick_x, 0.05);

        pod.updateModule(forwardScalar, sideScalar);

        SwervePod.ModuleState state = pod.getModuleState();

        telemetry.addData("Inputs to SwervePod", "");
        telemetry.addData("  forwardScalar", "%.3f", forwardScalar);
        telemetry.addData("  sideScalar", "%.3f", sideScalar);
        telemetry.addData("  turnScalar", "%.3f", turnScalar);
        telemetry.addData("", "");
        telemetry.addData("Pod State", "");
        telemetry.addData("  Current Heading", "%.1f°", Math.toDegrees(state.currentHeading));
        telemetry.addData("  Target Heading", "%.1f°", Math.toDegrees(state.targetHeading));
        telemetry.addData("  Heading Error", "%.1f°", Math.toDegrees(state.getHeadingError()));
        telemetry.addData("  Drive Velocity", "%.3f m/s", state.velocity);
        telemetry.addData("", "");
        telemetry.addData("✓ PASS if", "heading tracks stick direction");
        telemetry.update();
    }
}
