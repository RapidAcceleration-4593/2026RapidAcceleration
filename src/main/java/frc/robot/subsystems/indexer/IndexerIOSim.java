package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class IndexerIOSim extends IndexerIOReal implements IPhysicsSim {

    private final SparkMaxSim spindexerMotorSim;
    private final SparkMaxSim feederMotorSim;
    private final DIOSim sensorSim;

    private final FlywheelSim spindexerSim;
    private final FlywheelSim feederSim;

    public IndexerIOSim() {
        DCMotor spindexerGearbox = DCMotor.getNeo550(1);
        DCMotor feederGearbox = DCMotor.getNEO(1);

        spindexerSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        spindexerGearbox, kSpindexerMOI.in(KilogramSquareMeters), kSpindexerGearing),
                spindexerGearbox);
        feederSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(feederGearbox, kFeederMOI.in(KilogramSquareMeters), kFeederGearing),
                feederGearbox);

        spindexerMotorSim = new SparkMaxSim(spindexerMotor, spindexerGearbox);
        feederMotorSim = new SparkMaxSim(feederMotor, feederGearbox);
        sensorSim = new DIOSim(sensor);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        spindexerSim.setInput(spindexerMotorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
        feederSim.setInput(feederMotorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
        spindexerSim.update(0.02);
        feederSim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(spindexerMotorSim.getMotorCurrent() + feederMotorSim.getMotorCurrent());
    }

    @Override
    public void updateIOSim() {
        spindexerMotorSim.iterate(
                spindexerSim.getAngularVelocityRPM(),
                SimulatedBattery.getBatteryVoltage().in(Volts),
                0.02);

        feederMotorSim.iterate(
                feederSim.getAngularVelocityRPM(),
                SimulatedBattery.getBatteryVoltage().in(Volts),
                0.02);

        // sensorSim.setValue();
    }
}
