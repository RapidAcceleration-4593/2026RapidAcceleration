package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public static class TurretInputs {
        public boolean atAngle = false;
        public Angle angle = Degrees.zero();
        public Angle targetAngle = Degrees.zero();

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(TurretInputs inputs) {}

    /** Applies the feedback control loop mechanism. */
    public default void updateControl() {}

    /** Sets the angle of the turret motor. */
    public default void setAngle(Angle angle) {}

    /** Stops the turret motor immediately. */
    public default void stop() {}
}
