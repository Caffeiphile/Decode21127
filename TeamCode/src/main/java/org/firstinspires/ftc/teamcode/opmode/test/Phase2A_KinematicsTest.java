package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveKinematics;
import org.firstinspires.ftc.teamcode.subsystems.swerve.Vector2d;

@TeleOp(name="Phase 2A: Kinematics Test", group="Swerve Test")
public class Phase2A_KinematicsTest extends OpMode {
    private SwerveKinematics kinematics;
    private int testCase = 0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    private static final String[] TEST_NAMES = {
            "Pure Forward",
            "Pure Strafe",
            "Pure Rotation",
            "Diagonal",
            "Arc Path"
    };

    @Override
    public void init() {
        Vector2d[] positions = new Vector2d[] {
                new Vector2d(SwerveConstants.WHEELBASE/2, -SwerveConstants.WHEELBASE/2),
                new Vector2d(-SwerveConstants.WHEELBASE/2, SwerveConstants.WHEELBASE/2)
        };

        kinematics = new SwerveKinematics(positions);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "D-pad Up/Down to cycle tests");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !dpadUpPressed) {
            testCase = (testCase + 1) % TEST_NAMES.length;
        }
        dpadUpPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPressed) {
            testCase = (testCase - 1 + TEST_NAMES.length) % TEST_NAMES.length;
        }
        dpadDownPressed = gamepad1.dpad_down;

        double vx = 0, vy = 0, omega = 0;

        switch (testCase) {
            case 0:
                vx = 1.0;
                break;
            case 1:
                vy = 1.0;
                break;
            case 2:
                omega = 1.0;
                break;
            case 3:
                vx = 1.0;
                vy = 1.0;
                break;
            case 4:
                vx = 1.0;
                omega = 0.5;
                break;
        }

        SwerveKinematics.ModuleState[] states = kinematics.calculateModuleStates(vx, vy, omega);

        telemetry.addData("Test Case", TEST_NAMES[testCase]);
        telemetry.addData("Input", "vx=%.1f vy=%.1f ω=%.1f", vx, vy, omega);
        telemetry.addData("", "");

        String[] names = {"Left", "Right"};
        for (int i = 0; i < states.length; i++) {
            telemetry.addData(names[i],
                    "heading=%.0f° vel=%.2f",
                    Math.toDegrees(states[i].heading),
                    states[i].velocity
            );
        }

        telemetry.update();
    }
}