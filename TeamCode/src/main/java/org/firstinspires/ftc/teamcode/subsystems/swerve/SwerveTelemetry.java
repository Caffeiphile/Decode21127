package org.firstinspires.ftc.teamcode.subsystems.swerve;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class SwerveTelemetry {
    private final Telemetry telemetry;
    private long lastUpdateTime = 0;
    private int updateRateMs = 100;

    public static final String[] MODULE_NAMES = {"krai", "cid"};

    public SwerveTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void displaySwerveState(SwerveBase swerve, double loopTimeMs) {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime < updateRateMs) {
            return;
        }
        lastUpdateTime = currentTime;

        telemetry.addData("Loop Time", "%.1f ms", loopTimeMs);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(swerve.getHeading()));
        telemetry.addData("Field-Centric", swerve.isFieldCentric() ? "ON" : "OFF");

        Pose2D pos = swerve.getPosition();
        telemetry.addData("Position", "X: %.2f Y: %.2f", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM));
        telemetry.addData("", "");

        SwervePod.ModuleState[] states = swerve.getModuleStates();

        for (int i = 0; i < states.length; i++) {
            SwervePod.ModuleState state = states[i];
            telemetry.addData(MODULE_NAMES[i],
                    "%.0f° | %.2f m/s | Err: %.1f°",
                    Math.toDegrees(state.currentHeading),
                    state.velocity,
                    Math.toDegrees(state.getHeadingError())
            );
        }

        telemetry.update();
    }

    public void setUpdateRate(int milliseconds) {
        this.updateRateMs = milliseconds;
    }

    public void displayWarning(String message) {
        telemetry.addData("WARNING", message);
    }
}

