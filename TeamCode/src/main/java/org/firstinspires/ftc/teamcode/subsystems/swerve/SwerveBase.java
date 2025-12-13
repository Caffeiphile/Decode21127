package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.List;

public class SwerveBase {
    private final SwervePod[] pods;
    private final SwerveKinematics kinematics;
    private final PinpointWrapper pinpoint;
    private final List<LynxModule> allHubs;

    private double cachedHeading = 0;
    private double cachedCosHeading = 1;
    private double cachedSinHeading = 0;
    private boolean fieldCentricEnabled = true;

    public SwerveKinematics.ModuleState[] states;

    public SwerveBase(HardwareMap hardwareMap, ModuleConfiguration[] configs, PinpointWrapper pinpoint) {
        this.pinpoint = pinpoint;
        this.pods = new SwervePod[configs.length];
        Vector2d[] positions = new Vector2d[configs.length];

        for (int i = 0; i < configs.length; i++) {
            SwerveModuleHardware hardware = new SwerveModuleHardware(hardwareMap, configs[i].topMotorName,
                    configs[i].bottomMotorName, configs[i].encoderName, configs[i].topMotorReversed,
                    configs[i].bottomMotorReversed, configs[i].encoderOffset);

            DriveMotorController driveController = new DriveMotorController(
                    SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA, SwerveConstants.DRIVE_KS);
            SteeringPIDController steeringController = new SteeringPIDController(
                    SwerveConstants.STEER_KP, SwerveConstants.STEER_KD, SwerveConstants.STEER_KI);

            pods[i] = new SwervePod(hardware, configs[i].position, driveController, steeringController);
            positions[i] = configs[i].position;
        }

        this.kinematics = new SwerveKinematics(positions);
        this.allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void drive(double vxField, double vyField, double omegaField) {
        bulkReadSensors();

        double vx, vy;
        if (fieldCentricEnabled) {
            vx = vxField * cachedCosHeading + vyField * cachedSinHeading;
            vy = -vxField * cachedSinHeading + vyField * cachedCosHeading;
        } else {
            vx = vxField;
            vy = vyField;
        }

        SwerveKinematics.ModuleState[] states = kinematics.calculateModuleStates(vx, vy, omegaField);
        kinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_WHEEL_SPEED);

        for (int i = 0; i < pods.length; i++) {
            pods[i].updateModule(
                    states[i].velocity,
                    states[i].heading
            );
        }
        this.states = states;
    }

    private void bulkReadSensors() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        pinpoint.update();
        cachedHeading = pinpoint.getHeading();
        cachedCosHeading = Math.cos(cachedHeading);
        cachedSinHeading = Math.sin(cachedHeading);
    }

    public void stop() {
        for (SwervePod pod : pods) {
            pod.updateModule(0, 0);
        }
    }

    public void resetHeading() { pinpoint.resetHeading(); }
    public void resetPosition() { pinpoint.resetPosition(); }
    public Pose2D getPosition() { return pinpoint.getPosition(); }
    public double getHeading() { return cachedHeading; }
    public void setFieldCentric(boolean enabled) { fieldCentricEnabled = enabled; }
    public boolean isFieldCentric() { return fieldCentricEnabled; }

    public SwervePod.ModuleState[] getModuleStates() {
        SwervePod.ModuleState[] states = new SwervePod.ModuleState[pods.length];
        for (int i = 0; i < pods.length; i++) {
            states[i] = pods[i].getModuleState();
        }
        return states;
    }
}