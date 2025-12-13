package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Phase 1A: Single Module Test", group="Swerve Test")
public class Phase1A_SingleModuleTest extends OpMode {
    private SwervePod pod;
    private ElapsedTime loopTimer;

    private boolean testDriveOnly = false;
    private boolean testSteerOnly = false;

    @Override
    public void init() {
        loopTimer = new ElapsedTime();

        SwerveModuleHardware hardware = new SwerveModuleHardware(
                hardwareMap,
                "cidTop",
                "cidBottom",
                "cidEncoder",
                false,
                false,
                0.0
        );

        DriveMotorController driveFF = new DriveMotorController(
                SwerveConstants.DRIVE_KV,
                SwerveConstants.DRIVE_KA,
                SwerveConstants.DRIVE_KS
        );

        SteeringPIDController steeringPID = new SteeringPIDController(
                SwerveConstants.STEER_KP,
                SwerveConstants.STEER_KD
        );

        Vector2d position = new Vector2d(-SwerveConstants.WHEELBASE / 2, 0);

        pod = new SwervePod(hardware, position, driveFF, steeringPID);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopTimer.reset();

        if (gamepad1.x) {
            testDriveOnly = !testDriveOnly;
        }
        if (gamepad1.y) {
            testSteerOnly = !testSteerOnly;
        }

        double driveCommand = -gamepad1.left_stick_y * SwerveConstants.MAX_WHEEL_SPEED;
        double targetHeading = gamepad1.right_stick_x * Math.PI;

        if (testDriveOnly) {
            driveCommand = 0;
        }
        if (testSteerOnly) {
            targetHeading = 0;
        }

        driveCommand = MathUtils.deadband(driveCommand, 0.05);

        double forwardScalar = driveCommand * Math.cos(targetHeading);
        double sideScalar = driveCommand * Math.sin(targetHeading);

        pod.updateModule(forwardScalar, sideScalar);

        SwervePod.ModuleState state = pod.getModuleState();

        telemetry.addData("Mode", testDriveOnly ? "STEER ONLY" :
                (testSteerOnly ? "DRIVE ONLY" : "FULL CONTROL"));
        telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(state.currentHeading));
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(state.targetHeading));
        telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(state.getHeadingError()));
        telemetry.addData("Drive Velocity", "%.2f m/s", state.velocity);
        telemetry.addData("Loop Time", "%.1f ms", loopTimer.milliseconds());
        telemetry.addData("", "");
        telemetry.addData("Controls", "Left Y: Drive | Right X: Heading");
        telemetry.addData("", "A: Reset | X: Steer Only | Y: Drive Only");
        telemetry.update();
    }
}