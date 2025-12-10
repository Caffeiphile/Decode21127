package org.firstinspires.ftc.teamcode.subsystems.swerve;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EncoderWrapper {
    private final AnalogInput encoder;
    private final double offsetRadians;
    private final boolean reversed;

    private static final double VOLTAGE_RANGE = 3.3;

    public EncoderWrapper(HardwareMap hardwareMap, String encoderName,
                          double offsetRadians, boolean reversed) {
        this.encoder = hardwareMap.get(AnalogInput.class, encoderName);
        this.offsetRadians = offsetRadians;
        this.reversed = reversed;
    }

    public double getAngle() {
        double voltage = encoder.getVoltage();
        double rawAngle = (voltage / VOLTAGE_RANGE) * 2 * Math.PI;

        if (reversed) {
            rawAngle = 2 * Math.PI - rawAngle;
        }

        double angle = rawAngle - offsetRadians;
        return MathUtils.normalizeAngle(angle);
    }

    public double getRawAngle() {
        double voltage = encoder.getVoltage();
        return (voltage / VOLTAGE_RANGE) * 2 * Math.PI;
    }

    public boolean isConnected() {
        try {
            encoder.getVoltage();
            return true;
        } catch (Exception e) {
            return false;
        }
    }
}
