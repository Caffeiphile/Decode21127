package org.firstinspires.ftc.teamcode.subsystems.swerve.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Pinpoint Offset Tuning", group="Tuning")
public class PinpointOffsetTuning extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    private ElapsedTime runtime;

    private double xOffset = SwerveConstants.PINPOINT_X_OFFSET;
    private double yOffset = SwerveConstants.PINPOINT_Y_OFFSET;

    private Pose2D startPos;
    private double testDistance = 1.0;

    private enum TestPhase {
        SETUP,
        FORWARD_TEST,
        STRAFE_TEST,
        ROTATION_TEST,
        COMPLETE
    }

    private TestPhase phase = TestPhase.SETUP;

    private double forwardError = 0;
    private double strafeError = 0;
    private double rotationError = 0;

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        runtime = new ElapsedTime();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "");
        telemetry.addData("1", "Adjust offsets with D-pad and bumpers");
        telemetry.addData("2", "Press A to run test sequence");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        lastA = gamepad1.a;
        lastB = gamepad1.b;

        pinpoint.update();

        if (phase == TestPhase.SETUP) {
            if (gamepad1.dpad_up) xOffset += 1.0;
            if (gamepad1.dpad_down) xOffset -= 1.0;
            if (gamepad1.dpad_right) yOffset += 1.0;
            if (gamepad1.dpad_left) yOffset -= 1.0;

            if (gamepad1.right_bumper) {
                xOffset += 10.0;
            }
            if (gamepad1.left_bumper) {
                xOffset -= 10.0;
            }

            if (gamepad1.right_trigger > 0.5) {
                yOffset += 10.0;
            }
            if (gamepad1.left_trigger > 0.5) {
                yOffset -= 10.0;
            }

            pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);

            if (aPressed) {
                phase = TestPhase.FORWARD_TEST;
                pinpoint.resetPosAndIMU();
                startPos = pinpoint.getPosition();
                telemetry.addData("Test", "Drive forward 1 MM, then press B");
                telemetry.update();
            }
        }

        if (phase == TestPhase.FORWARD_TEST && bPressed) {
            Pose2D currentPos = pinpoint.getPosition();
            forwardError = Math.abs(currentPos.getX(DistanceUnit.MM) - testDistance);

            phase = TestPhase.STRAFE_TEST;
            pinpoint.resetPosAndIMU();
            telemetry.addData("Test", "Strafe left 1 MM, then press B");
            telemetry.update();
        }

        if (phase == TestPhase.STRAFE_TEST && bPressed) {
            Pose2D currentPos = pinpoint.getPosition();
            strafeError = Math.abs(currentPos.getY(DistanceUnit.MM) - testDistance);

            phase = TestPhase.ROTATION_TEST;
            pinpoint.resetPosAndIMU();
            telemetry.addData("Test", "Rotate 360°, then press B");
            telemetry.update();
        }

        if (phase == TestPhase.ROTATION_TEST && bPressed) {
            Pose2D currentPos = pinpoint.getPosition();
            rotationError = Math.abs(currentPos.getHeading(AngleUnit.DEGREES));

            phase = TestPhase.COMPLETE;
        }

        if (phase == TestPhase.COMPLETE && aPressed) {
            phase = TestPhase.SETUP;
        }

        displayTelemetry();
    }

    private void displayTelemetry() {
        telemetry.addData("Phase", phase.toString());
        telemetry.addData("", "");

        telemetry.addData("Current Offsets (mm)", "");
        telemetry.addData("  X Offset", "%.1f (D-pad Up/Down, Bumpers ±10)", xOffset);
        telemetry.addData("  Y Offset", "%.1f (D-pad Left/Right, Triggers ±10)", yOffset);
        telemetry.addData("", "");

        Pose2D pos = pinpoint.getPosition();
        telemetry.addData("Current Position", "");
        telemetry.addData("  X", "%.3f m", pos.getX(DistanceUnit.MM));
        telemetry.addData("  Y", "%.3f m", pos.getY(DistanceUnit.MM));
        telemetry.addData("  Heading", "%.1f°", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("", "");

        if (phase == TestPhase.COMPLETE) {
            telemetry.addData("Test Results", "");
            telemetry.addData("Forward Error", "%.3f m %s",
                    forwardError, forwardError < 0.02 ? "✓" : "✗");
            telemetry.addData("Strafe Error", "%.3f m %s",
                    strafeError, strafeError < 0.02 ? "✓" : "✗");
            telemetry.addData("Rotation Error", "%.1f° %s",
                    rotationError, rotationError < 3.0 ? "✓" : "✗");
            telemetry.addData("", "");

            if (forwardError < 0.02 && strafeError < 0.02 && rotationError < 3.0) {
                telemetry.addData("Status", "✓ OFFSETS GOOD");
            } else {
                telemetry.addData("Status", "✗ ADJUST OFFSETS");

                if (forwardError >= 0.02) {
                    if (pos.getX(DistanceUnit.MM) > testDistance) {
                        telemetry.addData("Suggestion", "Increase X offset");
                    } else {
                        telemetry.addData("Suggestion", "Decrease X offset");
                    }
                }

                if (strafeError >= 0.02) {
                    if (pos.getY(DistanceUnit.MM) > testDistance) {
                        telemetry.addData("Suggestion", "Increase Y offset");
                    } else {
                        telemetry.addData("Suggestion", "Decrease Y offset");
                    }
                }
            }

            telemetry.addData("", "");
            telemetry.addData("Copy to SwerveConstants.java", "");
            telemetry.addData("PINPOINT_X_OFFSET", "%.1f", xOffset);
            telemetry.addData("PINPOINT_Y_OFFSET", "%.1f", yOffset);
            telemetry.addData("", "");
            telemetry.addData("Press A", "to run test again");
        }

        if (phase == TestPhase.SETUP) {
            telemetry.addData("Controls", "");
            telemetry.addData("D-pad", "Adjust offsets (±1mm)");
            telemetry.addData("Bumpers", "Adjust X offset (±10mm)");
            telemetry.addData("Triggers", "Adjust Y offset (±10mm)");
            telemetry.addData("A", "Start test sequence");
        }

        telemetry.update();
    }
}