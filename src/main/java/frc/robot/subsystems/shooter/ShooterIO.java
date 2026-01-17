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

    /** Sets velocity of the shooter motor in volts. */
    public default void setVelocity(double voltage) {}

    /** Returns the current shooter velocity in RPM. */
    public default double getVelocity() {
        return 0.0;
    }

    /** Returns true if the shooter is at speed. */
    public default boolean atSpeed() {
        return false;
    }
}
