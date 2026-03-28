package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class DeployIOSim extends DeployIOReal implements IPhysicsSim {

    private final ElevatorSim deploySim;
    private final SparkMaxSim motorSim;
    private final DIOSim retractedLSSim;

    public DeployIOSim() {
        DCMotor gearbox = DCMotor.getNeo550(2);

        deploySim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        gearbox, kCarriageMass.in(Kilograms), kDrumRadius.in(Meters), kMotorToDeployGearing),
                gearbox,
                kMinimumDistance.in(Meters),
                kMaximumDistance.in(Meters),
                false,
                kMinimumDistance.in(Meters));

        motorSim = new SparkMaxSim(motor, gearbox);
        retractedLSSim = new DIOSim(retractedLS);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        deploySim.setInput(motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
        deploySim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(motorSim.getMotorCurrent());
    }

    @Override
    public void updateIOSim() {
        LinearVelocity deployVelocity = MetersPerSecond.of(deploySim.getVelocityMetersPerSecond());
        motorSim.iterate(
                deployVelocity.in(InchesPerSecond) * 60,
                SimulatedBattery.getBatteryVoltage().in(Volts),
                0.02);

        retractedLSSim.setValue(deploySim.hasHitLowerLimit() ^ kInvertRetractedLS);
        SimulationManager.getInstance().setIntakeExtended(deploySim.hasHitUpperLimit());
    }
}
