package frc.robot.util;

import edu.wpi.first.units.measure.Current;

public interface IPhysicsSim {

    /** Updates the physical plant (motors, mechanisms, inertia, etc.). */
    public void updatePlantSim();

    /** Reports the current draw and other electrical to the power model. */
    public Current getCurrentDraw();

    /** Updates the simulated sensor values from the IO interfaces. */
    public void updateIOSim();
}
