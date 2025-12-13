package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.swerve.*;

@TeleOp(name="Swerve TeleOp", group="Swerve")
public class SwerveTeleOp extends OpMode {
    private SwerveBase swerve;
    private PinpointWrapper pinpoint;
    private ElapsedTime loopTimer;
    private double speedMultiplier = 0.7;

    @Override
    public void init() {
        loopTimer = new ElapsedTime();
        pinpoint = new PinpointWrapper(hardwareMap, "pinpoint");

        double offset = SwerveConstants.POD_OFFSET_FROM_CENTER;
        ModuleConfiguration[] configs = new ModuleConfiguration[] {
                ModuleConfiguration.createStandard("cid",
                        new Vector2d(-offset, -offset),-3.0978),
                ModuleConfiguration.createStandard("krai",
                        new Vector2d(offset, offset),-0.0952)
        };

        swerve = new SwerveBase(hardwareMap, configs, pinpoint);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopTimer.reset();

        if (gamepad1.options) swerve.resetHeading();
        if (gamepad1.share) swerve.resetPosition();
        if (gamepad1.dpad_up) swerve.setFieldCentric(!swerve.isFieldCentric());

        speedMultiplier = gamepad1.left_bumper ? 0.3 : (gamepad1.right_bumper ? 1.0 : 0.7);

        double vx = MathUtils.deadband(-gamepad1.left_stick_y, 0.05) * SwerveConstants.MAX_TRANSLATION_SPEED * speedMultiplier;
        double vy = MathUtils.deadband(-gamepad1.left_stick_x, 0.05) * SwerveConstants.MAX_TRANSLATION_SPEED * speedMultiplier;
        double omega = MathUtils.deadband(-gamepad1.right_stick_x, 0.05) * SwerveConstants.MAX_ROTATION_SPEED * speedMultiplier;

        swerve.drive(vx, vy, omega);

        Pose2D pos = swerve.getPosition();
        telemetry.addData("Loop Time", "%.1f ms", loopTimer.milliseconds());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(swerve.getHeading()));
        telemetry.addData("Position", "X: %.2f Y: %.2f", pos.getX(DistanceUnit.METER), pos.getY(DistanceUnit.METER));
        telemetry.addData("Field-Centric", swerve.isFieldCentric() ? "ON" : "OFF");
        telemetry.addData("Speed", speedMultiplier == 0.3 ? "PRECISION" : (speedMultiplier == 1.0 ? "TURBO" : "NORMAL"));

        SwervePod.ModuleState[] states = swerve.getModuleStates();
        telemetry.addData("Left", "%.0f° | %.2f m/s", Math.toDegrees(states[0].currentHeading), states[0].velocity);
        telemetry.addData("Right", "%.0f° | %.2f m/s", Math.toDegrees(states[1].currentHeading), states[1].velocity);

        telemetry.addData("Pod 0 raw value X: ", swerve.states[0].velocityVector.x);
        telemetry.addData("Pod 0 raw value Y: ", swerve.states[0].velocityVector.y);
        telemetry.addData("Pod 1 raw value X: ", swerve.states[1].velocityVector.x);
        telemetry.addData("Pod 1 raw value Y: ", swerve.states[1].velocityVector.y);

        telemetry.update();
    }

    @Override
    public void stop() {
        swerve.stop();
    }
}