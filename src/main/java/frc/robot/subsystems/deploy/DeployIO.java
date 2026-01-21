package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface DeployIO {

    @AutoLog
    public static class DeployInputs {
        public Distance distance = Inches.zero();
        public Distance targetDistance = Inches.zero();

        public boolean atTargetDistance = false;
        public boolean isLimitSwitchPressed = false;

        public Voltage leftAppliedVolts = Volts.zero();
        public Voltage rightAppliedVolts = Volts.zero();

        public Current leftOutputCurrent = Amps.zero();
        public Current rightOutputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(DeployInputs inputs) {}

    /** Applies the feedback control loop mechanism. */
    public default void updateControl() {}

    /** Sets the distance of the deploy in inches. */
    public default void setDistance(Distance distance) {}

    /** Stops the deployment mechanism immediately. */
    public default void stopDeploy() {}
}
