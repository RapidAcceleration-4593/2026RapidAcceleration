package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double targetRPM = 0.0;
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ShooterInputs inputs) {}

    /** Sets the shooter motor speed. Positive velocities shoot outward. */
    public default void setTargetVelocity(double rpm) {}

    /** Stops the shooter motor immediately. */
    public default void stop() {}
}
