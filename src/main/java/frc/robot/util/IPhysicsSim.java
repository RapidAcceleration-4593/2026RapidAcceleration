package frc.robot.util;

public interface IPhysicsSim {

    /** Updates the physical plant (motors, mechanisms, inertia, etc.). */
    public void updatePlantSim();

    /** Reports the current draw and other electrical to the power model. */
    public void updatePowerSim();

    /** Updates the simulated sensor values from the IO interfaces. */
    public void updateIOSim();
}
