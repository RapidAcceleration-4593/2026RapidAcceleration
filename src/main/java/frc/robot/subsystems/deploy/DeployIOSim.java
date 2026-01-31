package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.SimulationManager;

public class DeployIOSim extends DeployIOReal implements IPhysicsSim {

    private final ElevatorSim deploySim;
    private final SparkMaxSim motorSim;
    private final SparkMaxAlternateEncoderSim encoderSim;
    private final DIOSim inLimitSwitchSim;
    private final DIOSim outLimitSwitchSim;

    public DeployIOSim() {
        DCMotor gearbox = DCMotor.getNeo550(2);

        deploySim = new ElevatorSim(
                gearbox,
                kMotorToDeployGearing,
                kCarriageMass.in(Kilograms),
                kDrumRadius.in(Meters),
                kRetractedDistance.in(Meters),
                kExtendedDistance.in(Meters),
                false,
                kRetractedDistance.in(Meters));

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new SparkMaxAlternateEncoderSim(motor);
        inLimitSwitchSim = new DIOSim(inLimitSwitch);
        outLimitSwitchSim = new DIOSim(outLimitSwitch);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        double input = motorSim.getAppliedOutput() * PowerSim.getRailVoltage().in(Volts);
        deploySim.setInput(input);
        deploySim.update(0.02);
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.of(motorSim.getMotorCurrent()));
    }

    @Override
    public void updateIOSim() {
        var carriageMPS = deploySim.getVelocityMetersPerSecond();
        var drumRadPS = carriageMPS / kDrumRadius.in(Meters);
        AngularVelocity motorAngularVelocity = RadiansPerSecond.of(drumRadPS * kMotorToDeployGearing);
        motorSim.iterate(motorAngularVelocity.in(RPM), PowerSim.getRailVoltage().in(Volts), 0.02);
        encoderSim.setPosition(deploySim.getPositionMeters());
        inLimitSwitchSim.setValue(deploySim.hasHitLowerLimit());
        outLimitSwitchSim.setValue(deploySim.hasHitUpperLimit());
    }
}
