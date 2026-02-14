package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kZeroVelocity;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterInputs {
        public AngularVelocity velocity = kZeroVelocity;
        public AngularVelocity targetVelocity = kZeroVelocity;

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ShooterInputs inputs) {}

    /** Sets the velocity of the shooter motor. */
    public default void setVelocity(AngularVelocity velocity) {}

    /** Sets the voltage of the shooter motor. */
    public default void setVoltage(Voltage volts) {}

    /** Stops the shooter motor immediately. */
    public default void stop() {}
}
