package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class DeployIOSim extends DeployIOReal implements IPhysicsSim {

    private final ElevatorSim deploySim;

    private final SparkMaxSim motorSim;
    private final SparkMaxAlternateEncoderSim encoderSim;
    private final DIOSim retractedLSSim;

    public DeployIOSim() {
        DCMotor gearbox = DCMotor.getNeo550(1);

        deploySim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        gearbox, kCarriageMass.in(Kilograms), kDrumRadius.in(Meters), kMotorToDeployGearing),
                gearbox,
                kMinimumDistance.in(Meters),
                kMaximumDistance.in(Meters),
                false,
                kMinimumDistance.in(Meters));

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new SparkMaxAlternateEncoderSim(motor);
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
        LinearVelocity carriageVelocity = MetersPerSecond.of(deploySim.getVelocityMetersPerSecond());
        AngularVelocity drumVelocity =
                RadiansPerSecond.of(carriageVelocity.in(MetersPerSecond) / kDrumRadius.in(Meters));
        AngularVelocity motorVelocity = drumVelocity.times(kMotorToDeployGearing);
        motorSim.iterate(
                motorVelocity.in(RPM), SimulatedBattery.getBatteryVoltage().in(Volts), 0.02);

        Distance deployDistance = Meters.of(deploySim.getPositionMeters());
        encoderSim.setPosition(deployDistance.in(Inches));

        retractedLSSim.setValue(deploySim.hasHitLowerLimit() ^ kInvertRetractedLS);
        SimulationManager.getInstance().setIntakeExtended(deploySim.hasHitUpperLimit());
    }
}
