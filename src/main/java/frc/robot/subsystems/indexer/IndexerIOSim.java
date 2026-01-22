package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.PowerSim;

public class IndexerIOSim extends IndexerIOReal {

    private final SparkMaxSim spindexerSim;
    private final SparkMaxSim feederSim;

    private final DCMotor spindexerGearbox;
    private final DCMotor feederGearbox;

    public IndexerIOSim() {
        spindexerGearbox = DCMotor.getNeo550(1);
        feederGearbox = DCMotor.getNEO(1);

        spindexerSim = new SparkMaxSim(spindexerMotor, spindexerGearbox);
        feederSim = new SparkMaxSim(feederMotor, feederGearbox);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        super.updateInputs(inputs);
        updateSimulation();
    }

    /** Updates simulation variables periodically. */
    private void updateSimulation() {
        PowerSim.addCurrentDraw(Amps.of(spindexerSim.getMotorCurrent()));
        PowerSim.addCurrentDraw(Amps.of(feederSim.getMotorCurrent()));
    }
}
