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

    /** Sets the speed of the intake mechanism as a percentage. */
    public default void setIntakeSpeed(double speed) {}

    /** Stops the intake motor immediately. */
    public default void stopIntake() {}
}
