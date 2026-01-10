package frc.robot.util;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.simulation.BatterySim;

public class PowerSim {
    private static double totalCurrentAmps = 0;

    public static void addCurrentDraw(double amps) {
        totalCurrentAmps += amps;
    }

    public static double getRailVoltage() {
        return RoboRioDataJNI.getVInVoltage();
    }

    public static void simulationPeriodic() {
        var voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentAmps);
        RoboRioDataJNI.setVInVoltage(voltage);
        totalCurrentAmps = 0;
    }
}
