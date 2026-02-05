package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public static class ClimberInputs {
        public Distance distance = kMinimumDistance;

        public Voltage leftAppliedVolts = Volts.zero();
        public Current leftOutputCurrent = Amps.zero();

        public Voltage rightAppliedVolts = Volts.zero();
        public Current rightOutputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ClimberInputs inputs) {}

    /** Sets the voltage for the left climber. */
    public default void setLeftVoltage(Voltage volts) {}

    /** Sets the voltage for the right climber. */
    public default void setRightVoltage(Voltage volts) {}

    /** Stops the left climber motor immediately. */
    public default void stopLeft() {}

    /** Stops the right climber motor immediately. */
    public default void stopRight() {}
}
