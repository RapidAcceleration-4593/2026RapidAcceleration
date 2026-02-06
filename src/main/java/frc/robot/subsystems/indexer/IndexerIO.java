package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerInputs {
        public AngularVelocity spindexerVelocity = RPM.zero();
        public AngularVelocity feederVelocity = RPM.zero();

        public Voltage spindexerVolts = Volts.zero();
        public Voltage feederVolts = Volts.zero();

        public Current spindexerCurrent = Amps.zero();
        public Current feederCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(IndexerInputs inputs) {}

    /** Sets the voltage of the spindexer motor. */
    public default void setSpindexerVoltage(Voltage volts) {}

    /** Sets the voltage of the feeder motor. */
    public default void setFeederVoltage(Voltage volts) {}

    /** Stops the spindexer motor immediately. */
    public default void stopSpindexer() {}

    /** Stops the feeder motor immediately. */
    public default void stopFeeder() {}
}
