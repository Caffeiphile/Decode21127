package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Test 5: Motor Powers", group="Swerve Test")
public class Test5_MotorPowers extends OpMode {
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
        pod = new SwervePod(hardware, new Vector2d(-offset, -offset), driveController, steeringController);

        telemetry.addData("Test", "Final motor power outputs");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forwardScalar = MathUtils.deadband(-gamepad1.left_stick_y, 0.05);
        double sideScalar = MathUtils.deadband(gamepad1.left_stick_x, 0.05);
        double turnScalar = MathUtils.deadband(gamepad1.right_stick_x, 0.05);

        pod.updateModule(forwardScalar, sideScalar);

        SwervePod.ModuleState state = pod.getModuleState();

        telemetry.addData("Motor Powers", "");
        telemetry.addData("  Top Motor", "%.3f", state.topMotorPower);
        telemetry.addData("  Bottom Motor", "%.3f", state.bottomMotorPower);
        telemetry.addData("", "");
        telemetry.addData("Module State", "");
        telemetry.addData("  Heading", "%.1f°", Math.toDegrees(state.currentHeading));
        telemetry.addData("  Target", "%.1f°", Math.toDegrees(state.targetHeading));
        telemetry.addData("  Error", "%.1f°", Math.toDegrees(state.getHeadingError()));
        telemetry.addData("", "");
        telemetry.addData("✓ PASS if", "motors spin when stick moved");
        telemetry.addData("✓ PASS if", "both motors same sign for drive");
        telemetry.addData("✓ PASS if", "motors opposite for steering");
        telemetry.update();
    }
}