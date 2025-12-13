package org.firstinspires.ftc.teamcode.opmode.teleOp;


import static org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveConstants.POD_OFFSET_FROM_CENTER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name = "Swerve Module Test", group = "Test")
public class SwerveModuleTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        // CHANGE THESE NAMES FOR THE MODULE YOU ARE TESTING
        SwerveModuleHardware hardware =
                new SwerveModuleHardware(
                        hardwareMap,
                        "cidTop",
                        "cidBottom",
                        "cidEncoder",
                        true,
                        false,
                        -3.0978
                );

        SteeringPIDController steerPID =
                new SteeringPIDController(0.80, 0.012, 0.0);

        SimpleSwerveModule module =
                new SimpleSwerveModule(hardware, steerPID);

        waitForStart();

        while (opModeIsActive()) {

            // Left stick Y → drive
            double drive = -gamepad1.left_stick_y * 0.7;

            // Right stick X → steering target
            double targetAngle = gamepad1.right_stick_x * Math.PI;

            module.set(drive, targetAngle);

            telemetry.addData("Drive", drive);
            telemetry.addData("Pod Speed", module.podSpeed);
            telemetry.addData("Pod Error", module.podError);
            telemetry.addData("Target Angle (rad)", targetAngle);
            telemetry.addData("Current Angle (rad)", hardware.getModuleAngle());
            telemetry.update();
        }
    }
}
