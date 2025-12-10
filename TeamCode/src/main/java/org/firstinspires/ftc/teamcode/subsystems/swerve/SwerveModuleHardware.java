package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class SwerveModuleHardware {
    private final DcMotor topMotor;
    private final DcMotor bottomMotor;
    private final EncoderWrapper encoder;
    private final VoltageSensor voltageSensor;

    public SwerveModuleHardware(HardwareMap hardwareMap,
                                String topMotorName, String bottomMotorName,
                                String encoderName,
                                boolean topMotorReversed, boolean bottomMotorReversed,
                                double encoderOffset) {
        this.topMotor = hardwareMap.get(DcMotor.class, topMotorName);
        this.bottomMotor = hardwareMap.get(DcMotor.class, bottomMotorName);

        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topMotor.setDirection(topMotorReversed ?
                DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        bottomMotor.setDirection(bottomMotorReversed ?
                DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        this.encoder = new EncoderWrapper(hardwareMap, encoderName, encoderOffset, false);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void setMotorPowers(double topPower, double bottomPower) {
        topMotor.setPower(MathUtils.clamp(topPower, -1.0, 1.0));
        bottomMotor.setPower(MathUtils.clamp(bottomPower, -1.0, 1.0));
    }

    public double getModuleAngle() {
        return encoder.getAngle();
    }

    public double getBatteryVoltage() {
        return voltageSensor.getVoltage();
    }

    public boolean isInitialized() {
        return topMotor != null && bottomMotor != null && encoder.isConnected();
    }

    @Override
    public String toString() {
        return String.format("Angle: %.1f°", Math.toDegrees(getModuleAngle()));
    }
}
