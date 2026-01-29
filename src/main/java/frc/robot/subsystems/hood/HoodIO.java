package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public static class HoodInputs {
        public Angle angle = Degrees.zero();
        public boolean limitswitch = false;

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(HoodInputs inputs) {}

    /** Sets voltage of the adjustable hood motor. */
    public default void setVoltage(Voltage volts) {}

    /** Stops the hood motor immediately. */
    public default void stop() {}

    /** Resets the hood encoder to zero position. */
    public default void resetEncoder() {}
}
