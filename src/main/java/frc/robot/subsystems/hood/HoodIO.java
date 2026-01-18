package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public static class HoodInputs {
        public double angle = 0.0;
        public boolean limitswitch = false;
        public double appliedVolts = 0.0;
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(HoodInputs inputs) {}

    /** Applies the feedback control loop mechanism. */
    public default void updateControl() {}

    /** Sets angle of the adjustable hood in Degrees. */
    public default void setAngle(double degrees) {}

    /** Returns the current angle of the adjustable hood in Degrees. */
    public default double getAngle() {
        return 0.0;
    }

    /** Returns true if the adjustable hood is at its setpoint angle. */
    public default boolean atAngle() {
        return false;
    }

    /** Stops the shooter motor immediately. */
    public default void stop() {}

    /** Resets the encoder reading to zero. */
    public default void resetEncoder() {}

    /** Retrieves whether the limit switch is pressed. */
    public default boolean getLimitSwitch() {
        return false;
    }
}
