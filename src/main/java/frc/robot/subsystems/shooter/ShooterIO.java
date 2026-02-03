package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterInputs {
        public AngularVelocity velocity = RPM.zero();
        public AngularVelocity targetVelocity = RPM.zero();

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ShooterInputs inputs) {}

    /** Sets the velocity of the shooter motor. */
    public default void setVelocity(AngularVelocity velocity) {}

    /** Stops the shooter motor immediately. */
    public default void stop() {}
}
