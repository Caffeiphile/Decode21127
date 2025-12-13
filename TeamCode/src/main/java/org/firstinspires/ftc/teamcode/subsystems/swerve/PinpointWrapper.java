package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


public class PinpointWrapper {
    private final GoBildaPinpointDriver pinpoint;
    private double headingOffset = 0;

    public PinpointWrapper(HardwareMap hardwareMap, String deviceName) {
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);

        pinpoint.setOffsets(
                SwerveConstants.PINPOINT_X_OFFSET,
                SwerveConstants.PINPOINT_Y_OFFSET,
                DistanceUnit.MM
        );

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
    }

    public double getHeading() {
        Pose2D pos = pinpoint.getPosition();
        return MathUtils.normalizeAngle(pos.getHeading(AngleUnit.RADIANS) - headingOffset);
    }

    public double getAngularVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public void resetHeading() {
        Pose2D currentPos = pinpoint.getPosition();
        headingOffset = currentPos.getHeading(AngleUnit.RADIANS);
    }

    public Pose2D getPosition() {
        return pinpoint.getPosition();
    }

    public double getVelX(DistanceUnit unit) {
        return pinpoint.getVelX(unit);
    }

    public double getVelY(DistanceUnit unit) {
        return pinpoint.getVelY(unit);
    }

    public void resetPosition() {
        pinpoint.resetPosAndIMU();
        headingOffset = 0;
    }

    public void setPosition(double x, double y, double heading) {
        pinpoint.resetPosAndIMU();
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeading());
    }

    public boolean isCalibrated() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }
}
