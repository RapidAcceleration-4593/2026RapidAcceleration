package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;

public class IndexerIOSim extends IndexerIOReal implements IPhysicsSim {

    private final SparkMaxSim spindexerSim;
    private final SparkMaxSim feederSim;

    public IndexerIOSim() {
        DCMotor spindexerGearbox = DCMotor.getNeo550(1);
        DCMotor feederGearbox = DCMotor.getNEO(1);

        spindexerSim = new SparkMaxSim(spindexerMotor, spindexerGearbox);
        feederSim = new SparkMaxSim(feederMotor, feederGearbox);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        // TODO: Implement MATTHEW.
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(spindexerSim.getMotorCurrent() + feederSim.getMotorCurrent());
    }

    @Override
    public void updateIOSim() {
        // TODO: Implement MATTHEW.
    }
}
