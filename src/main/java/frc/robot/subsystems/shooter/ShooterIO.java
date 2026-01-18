package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterInputs {
        public boolean atSpeed = false;
        public AngularVelocity velocity = RPM.zero();
        public AngularVelocity targetVelocity = RPM.zero();

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ShooterInputs inputs) {}

    /** Applies the feedback control loop mechanism. */
    public default void updateControl() {}

    /** Sets velocity of the shooter motor in RPM. */
    public default void setVelocity(AngularVelocity velocity) {}

    /** Returns the current shooter velocity in RPM. */
    public default AngularVelocity getVelocity() {
        return RPM.zero();
    }

    /** Returns true if the shooter is at speed. */
    public default boolean atSpeed() {
        return false;
    }

    /** Stops the shooter motor immediately. */
    public default void stop() {}
}
