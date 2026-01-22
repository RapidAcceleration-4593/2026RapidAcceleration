package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;

public final class PowerSim {

    /** Total current draw at the moment. */
    private static Current totalCurrent = Amps.zero();

    private PowerSim() {
        totalCurrent = Amps.zero();
    }

    /** Adds the current draw from a simulated subsystem. */
    public static void addCurrentDraw(Current amps) {
        totalCurrent = totalCurrent.plus(amps);
    }

    /** Retrieves the current simulated RoboRIO voltage. */
    public static Voltage getRailVoltage() {
        return Volts.of(RoboRioDataJNI.getVInVoltage());
    }

    public static void simulationPeriodic() {
        double voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent.in(Amps));
        RoboRioDataJNI.setVInVoltage(voltage);
        totalCurrent = Amps.zero();
    }
}
