package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public static class ClimberInputs {
        public Distance distance = Inches.zero();

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ClimberInputs inputs) {}

    /** Sets the voltage for the climber. */
    public default void setVoltage(Voltage volts) {}

    /** Stops the climber motor. */
    public default void stop() {}
}
