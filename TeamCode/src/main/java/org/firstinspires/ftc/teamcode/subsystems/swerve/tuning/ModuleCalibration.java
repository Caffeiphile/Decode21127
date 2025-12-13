package org.firstinspires.ftc.teamcode.subsystems.swerve.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Module Calibration", group="Tuning")
public class ModuleCalibration extends OpMode {
    private SwerveModuleHardware leftModule;
    private SwerveModuleHardware rightModule;

    private double leftOffset = 0;
    private double rightOffset = 0;

    private boolean lastA = false;

    @Override
    public void init() {
        leftModule = new SwerveModuleHardware(
                hardwareMap,
                "kraiTop",
                "kraiBottom",
                "kraiEncoder",
                true,
                true,
                0.0
        );

        rightModule = new SwerveModuleHardware(
                hardwareMap,
                "cidTop",
                "cidBottom",
                "cidEncoder",
                true,
                true,
                0.0
        );

        telemetry.addData("Status", "Ready");
        telemetry.addData("", "");
        telemetry.addData("Instructions", "");
        telemetry.addData("1", "Manually align both modules forward");
        telemetry.addData("2", "Ensure they point toward robot front");
        telemetry.addData("3", "Press A to capture offsets");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a && !lastA;
        lastA = gamepad1.a;

        double leftRaw = getLeftRawAngle();
        double rightRaw = getRightRawAngle();

        if (aPressed) {
            leftOffset = leftRaw;
            rightOffset = rightRaw;
        }

        double leftCorrected = leftRaw - leftOffset;
        double rightCorrected = rightRaw - rightOffset;

        telemetry.addData("Left Module", "");
        telemetry.addData("  Raw Angle", "%.1f°", Math.toDegrees(leftRaw));
        telemetry.addData("  Offset", "%.4f rad (%.1f°)", leftOffset, Math.toDegrees(leftOffset));
        telemetry.addData("  Corrected", "%.1f°", Math.toDegrees(leftCorrected));
        telemetry.addData("", "");

        telemetry.addData("Right Module", "");
        telemetry.addData("  Raw Angle", "%.1f°", Math.toDegrees(rightRaw));
        telemetry.addData("  Offset", "%.4f rad (%.1f°)", rightOffset, Math.toDegrees(rightOffset));
        telemetry.addData("  Corrected", "%.1f°", Math.toDegrees(rightCorrected));
        telemetry.addData("", "");

        if (leftOffset != 0 || rightOffset != 0) {
            telemetry.addData("Copy to Configuration", "");
            telemetry.addData("", "");
            telemetry.addData("Left Module:", "");
            telemetry.addData("  encoderOffset", "%.4f", leftOffset);
            telemetry.addData("", "");
            telemetry.addData("Right Module:", "");
            telemetry.addData("  encoderOffset", "%.4f", rightOffset);
            telemetry.addData("", "");

            double maxError = Math.max(
                    Math.abs(leftCorrected),
                    Math.abs(rightCorrected)
            );

            if (maxError < Math.toRadians(2)) {
                telemetry.addData("Status", "✓ GOOD - Offsets look correct");
            } else if (maxError < Math.toRadians(5)) {
                telemetry.addData("Status", "⚠ OK - Consider realigning");
            } else {
                telemetry.addData("Status", "✗ BAD - Modules not aligned forward");
            }
        } else {
            telemetry.addData("Status", "Press A when modules aligned forward");
        }

        telemetry.update();
    }

    private double getLeftRawAngle() {
        return leftModule.getModuleAngle();
    }

    private double getRightRawAngle() {
        return rightModule.getModuleAngle();
    }
}
