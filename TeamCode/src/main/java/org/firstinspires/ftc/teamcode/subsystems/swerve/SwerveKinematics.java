package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class SwerveKinematics {
    private final Vector2d[] modulePositions;

    public SwerveKinematics(Vector2d[] modulePositions) {
        this.modulePositions = modulePositions;
    }

    public ModuleState[] calculateModuleStates(double vx, double vy, double omega) {
        ModuleState[] states = new ModuleState[modulePositions.length];

        for (int i = 0; i < modulePositions.length; i++) {
            Vector2d pos = modulePositions[i];

            double vmx = vx - omega * pos.y;
            double vmy = vy + omega * pos.x;

            double velocity = Math.hypot(vmx, vmy);
            double heading = Math.atan2(vmy, vmx);

            states[i] = new ModuleState(velocity, heading, 0, new Vector2d(vmx, vmy));
        }

        return states;
    }

    public ModuleStateWithAccel[] calculateModuleStatesWithAccel(
            double vx, double vy, double omega,
            double ax, double ay, double alpha) {
        ModuleStateWithAccel[] states = new ModuleStateWithAccel[modulePositions.length];

        double omegaSq = omega * omega;

        for (int i = 0; i < modulePositions.length; i++) {
            Vector2d pos = modulePositions[i];

            double vmx = vx - omega * pos.y;
            double vmy = vy + omega * pos.x;

            double velocity = Math.hypot(vmx, vmy);
            double heading = Math.atan2(vmy, vmx);

            double amx = ax - alpha * pos.y - omegaSq * pos.x;
            double amy = ay + alpha * pos.x - omegaSq * pos.y;

            double acceleration = 0;
            if (velocity > 0.001) {
                acceleration = (amx * vmx + amy * vmy) / velocity;
            }

            states[i] = new ModuleStateWithAccel(
                    velocity, heading, acceleration, new Vector2d(vmx, vmy)
            );
        }

        return states;
    }

    public void desaturateWheelSpeeds(ModuleState[] states, double maxSpeed) {
        double maxVelocity = 0;

        for (ModuleState state : states) {
            maxVelocity = Math.max(maxVelocity, Math.abs(state.velocity));
        }

        if (maxVelocity > maxSpeed) {
            double scale = maxSpeed / maxVelocity;

            for (ModuleState state : states) {
                state.velocity *= scale;
                if (state instanceof ModuleStateWithAccel) {
                    ((ModuleStateWithAccel) state).acceleration *= scale;
                }
            }
        }
    }

    public static class ModuleState {
        public double velocity;
        public double heading;
        public double steeringVelocity;
        public Vector2d velocityVector;

        public ModuleState(double velocity, double heading,
                           double steeringVelocity, Vector2d velocityVector) {
            this.velocity = velocity;
            this.heading = heading;
            this.steeringVelocity = steeringVelocity;
            this.velocityVector = velocityVector;
        }
    }

    public static class ModuleStateWithAccel extends ModuleState {
        public double acceleration;

        public ModuleStateWithAccel(double velocity, double heading,
                                    double acceleration, Vector2d velocityVector) {
            super(velocity, heading, 0, velocityVector);
            this.acceleration = acceleration;
        }
    }
}
