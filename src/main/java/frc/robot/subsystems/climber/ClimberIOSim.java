package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.SimulationManager;

public class ClimberIOSim extends ClimberIOReal implements IPhysicsSim {

    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;

    private final DCMotor gearbox;

    public ClimberIOSim() {
        gearbox = DCMotor.getNEO(1);

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new EncoderSim(encoder);

        SimulationManager.getInstance().addSimulatable(this);
    }

    public void updateInputs(ClimberInputs inputs) {
        super.updateInputs(inputs);
    }

    @Override
    public void updatePlantSim() {}

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.zero());
    }

    @Override
    public void updateIOSim() {
        motorSim.iterate(0.0, PowerSim.getRailVoltage().in(Volts), 0.02);
        encoderSim.setDistance(0.0);
    }
}
