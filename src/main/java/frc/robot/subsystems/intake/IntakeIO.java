package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeInputs {
        public boolean isIntaking = false;

        public boolean isDeployed = false;
        public boolean isRetracted = false;

        // public Distance deployDistance = Inches.zero();

        public Voltage intakeVolts = Volts.zero();
        public Current intakeCurrent = Amps.zero();

        public Voltage deployVolts = Volts.zero();
        public Current deployCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(IntakeInputs inputs) {}

    /** Sets the speed of the intake mechanism as a percentage. */
    public default void setIntakeSpeed(double speed) {}

    /** Stops the intake motor immediately. */
    public default void stopIntake() {}

    /** Sets the speed of the deploy mechanism as a percentage. */
    public default void setDeploySpeed(double speed) {}

    /** Stops the deploy motor immediately. */
    public default void stopDeploy() {}
}
