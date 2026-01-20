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
        public Angle targetAngle = Degrees.zero();
        public boolean atTargetAngle = false;
        public boolean limitswitch = false;

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(HoodInputs inputs) {}

    /** Applies the feedback control loop mechanism. */
    public default void updateControl() {}

    /** Sets angle of the adjustable hood in Degrees. */
    public default void setAngle(Angle angle) {}

    /** Returns the current angle of the adjustable hood in Degrees. */
    public default Angle getAngle() {
        return Degrees.zero();
    }

    /** Returns true if the adjustable hood is at its setpoint angle. */
    public default boolean atAngle() {
        return false;
    }

    /** Stops the shooter motor immediately. */
    public default void stop() {}

    /** Resets the encoder reading to zero. */
    public default void resetEncoder() {}
}
