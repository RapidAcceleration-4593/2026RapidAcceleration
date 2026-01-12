package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public static class TurretInputs {
        /** The rotation of the turret; zero connotes default position. */
        public Angle angle = Degrees.of(0.0);

        /** The angular velocity of the turret in degrees/second. */
        public AngularVelocity angularVelocity = DegreesPerSecond.of(0.0);
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(TurretInputs inputs) {}

    /** Sets the speed of the turret motor. Positive velocities spin clockwise. */
    public default void setMotorSpeed(double speed) {}

    /** Stops the turret motor immediately. */
    public default void stopMotor() {}

    /** Resets the encoder value to zero. */
    public default void resetEncoder() {}
}
