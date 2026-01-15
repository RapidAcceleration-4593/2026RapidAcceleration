package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterInputs {}

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(ShooterInputs inputs) {}

    /** Sets the shooter motor speed. Positive velocities shoot outward. */
    public default void setMotorSpeed(double speed) {}

    /** Stops the shooter motor immediately. */
    public default void stopMotor() {}
}
