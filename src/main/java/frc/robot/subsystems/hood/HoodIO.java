package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public static class HoodInputs {
        public Angle angle = kMinimumAngle;
        public Angle targetAngle = kMinimumAngle;
        public boolean atTargetAngle = false;

        public boolean limitswitch = false;

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(HoodInputs inputs) {}

    /** Sets the position of the hood motor. */
    public default void setPosition(Angle angle) {}

    /** Resets the closed-loop error, encoder, and setpoint. */
    public default void resetPosition() {}

    /** Stops the hood motor immediately. */
    public default void stop() {}
}
