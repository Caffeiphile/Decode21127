package org.firstinspires.ftc.teamcode.opmode.teleOp;

import static org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveConstants.POD_OFFSET_FROM_CENTER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name = "Simple Swerve TeleOp", group = "Drive")
public class SimpleSwerveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        // === MODULE HARDWARE ===
        SimpleSwerveModule frontLeft = createModule(
                "cidTop",
                "cidBottom",
                "cidEncoder",
                true,
                false,
                -3.0978);
        SimpleSwerveModule backRight = createModule(
                "kraiTop",
                "kraiBottom",
                "kraiEncoder",
                true,
                false,
                -0.0952);

        SimpleSwerveModule[] modules = {
                frontLeft, backRight
        };

        // === MODULE POSITIONS (meters) ===
        Vector2d[] positions = {
                new Vector2d( POD_OFFSET_FROM_CENTER,  POD_OFFSET_FROM_CENTER), // FL
                new Vector2d(-POD_OFFSET_FROM_CENTER, -POD_OFFSET_FROM_CENTER)  // BR
        };

        SimpleSwerveDrive drive =
                new SimpleSwerveDrive(modules, positions);

        waitForStart();

        while (opModeIsActive()) {

            // FTC joystick convention
            double forward = -gamepad1.left_stick_y;
            double strafe  =  gamepad1.left_stick_x;
            double rotate  =  gamepad1.right_stick_x;

            // Scale for safety
            forward *= 0.8;
            strafe  *= 0.8;
            rotate  *= 0.6;

            drive.drive(forward, strafe, rotate);

            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Rotate", rotate);
            telemetry.update();
        }
    }

    // === Helper ===
    private SimpleSwerveModule createModule(
            String top, String bottom, String encoder,
            boolean topRev, boolean bottomRev, double offset) {

        SwerveModuleHardware hw =
                new SwerveModuleHardware(
                        hardwareMap,
                        top, bottom, encoder,
                        topRev, bottomRev, offset
                );

        SteeringPIDController pid =
                new SteeringPIDController(0.80, 0.012, 0.0);

        return new SimpleSwerveModule(hw, pid);
    }
}
