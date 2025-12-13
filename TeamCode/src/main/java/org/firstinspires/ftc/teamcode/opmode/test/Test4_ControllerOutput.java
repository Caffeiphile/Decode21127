package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Test 4: Controller Output", group="Swerve Test")
public class Test4_ControllerOutput extends OpMode {
    private SwerveModuleHardware hardware;
    private DriveMotorController driveController;
    private SteeringPIDController steeringController;

    @Override
    public void init() {
        hardware = new SwerveModuleHardware(hardwareMap, "cidTop", "cidBottom",
                "cidEncoder", true, true, -3.0978);
        driveController = new DriveMotorController(
                SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA, SwerveConstants.DRIVE_KS);
        steeringController = new SteeringPIDController(
                SwerveConstants.STEER_KP, SwerveConstants.STEER_KD, SwerveConstants.STEER_KI);

        telemetry.addData("Test", "Controller calculate() outputs");
        telemetry.update();
    }

    @Override
    public void loop() {
        double targetVel = MathUtils.deadband(-gamepad1.left_stick_y, 0.05) * 10.0;
        double currentHeading = hardware.getModuleAngle();
        double targetHeading = gamepad1.right_stick_x * Math.PI;

        double drivePower = driveController.calculate(targetVel, 0, hardware.getBatteryVoltage());
        double steeringOutput = steeringController.calculate(currentHeading, targetHeading);

        telemetry.addData("Drive Controller", "");
        telemetry.addData("  Input Velocity", "%.3f rad/s", targetVel);
        telemetry.addData("  Output Power", "%.3f", drivePower);
        telemetry.addData("", "");
        telemetry.addData("Steering Controller", "");
        telemetry.addData("  Current Heading", "%.1f°", Math.toDegrees(currentHeading));
        telemetry.addData("  Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        telemetry.addData("  Error", "%.1f°", Math.toDegrees(MathUtils.getShortestAngularDistance(currentHeading, targetHeading)));
        telemetry.addData("  Output", "%.3f rad/s", steeringOutput);
        telemetry.addData("", "");
        telemetry.addData("✓ PASS if", "drive power scales with stick");
        telemetry.addData("✓ PASS if", "steering output nonzero when error exists");
        telemetry.update();
    }
}
