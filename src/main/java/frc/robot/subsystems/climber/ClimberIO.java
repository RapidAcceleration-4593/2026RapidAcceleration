package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.kMinimumDistance;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    public static class ClimberInputs {
        public Distance distance = kMinimumDistance;
        public Distance targetDistance = kMinimumDistance;

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ClimberInputs inputs) {}

    /** Sets the position of the climber motor. */
    public default void setPosition(Distance distance) {}

    /** Sets the voltage for the climber. */
    public default void setVoltage(Voltage volts) {}

    /** Stops the climber motor immediately. */
    public default void stop() {}
}
