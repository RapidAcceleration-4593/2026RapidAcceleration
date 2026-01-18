package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

    @AutoLog
    public static class SpindexerInputs {
        public double spindexerVelocity = 0.0;
        public double feederVelocity = 0.0;

        public double spindexerCurrent = 0.0;
        public double feederCurrent = 0.0;

        public double spindexerVolts = 0.0;
        public double feederVolts = 0.0;
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(SpindexerInputs inputs) {}

    /** Sets the speed of the spindexer motor as a percentage. */
    public default void setSpindexerSpeed(double speed) {}

    /** Stops the spindexer motor immediately. */
    public default void stopSpindexer() {}

    /** Sets the speed of the feeder motor as a percentage. */
    public default void setFeederSpeed(double speed) {}

    /** Stops the feeder motor immediately. */
    public default void stopFeeder() {}
}
