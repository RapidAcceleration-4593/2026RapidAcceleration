package frc.robot.util;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public interface IPhysicsSim {
    /**
     * Updates physics models such as {@link ElevatorSim}. This is called before {@link #updatePowerSim()} and
     * {@link #updateIOSim()}.
     */
    public void updatePlantSim();

    /**
     * This is where calls to {@link PowerSim#addCurrentDraw(double)} should be done, ensuring all simulators have a
     * consistent power simulation. This is called after {@link #updatePowerSim()} and before {@link #updateIOSim()}.
     */
    public void updatePowerSim();

    /**
     * Updates outputs such as motors and solenoids, and inputs such as encoders, based off of the power and physics
     * simulations which were computed in previous steps. This is called after {@link #updatePowerSim()} and
     * {@link #updateIOSim()}.
     */
    public void updateIOSim();
}
