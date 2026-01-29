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
        public boolean inLimitSwitch = false;
        public boolean outLimitSwitch = false;

        public Voltage appliedVolts = Volts.zero();
        public Current outputCurrent = Amps.zero();
    }

    /** Fetches updates from sensors through the IO interface. */
    public default void updateInputs(DeployInputs inputs) {}

    /** Sets the voltage of the deployment motor. */
    public default void setVoltage(Voltage volts) {}

    /** Stops the deployment motor immediately. */
    public default void stop() {}

    /** Resets the deploy encoder to zero position. */
    public default void resetEncoder() {}
}
