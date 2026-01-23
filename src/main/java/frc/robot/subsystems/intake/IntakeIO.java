package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeInputs {
        public boolean isIntaking = false;

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(IntakeInputs inputs) {}

    /** Runs the intake motor. */
    public default void run() {}

    /** Stops the intake motor immediately. */
    public default void stop() {}
}
