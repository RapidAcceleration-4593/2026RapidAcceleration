package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.SimulationManager;

public class DeployIOSim extends DeployIOReal implements IPhysicsSim {

    private final ElevatorSim deploySim;
    private final SparkMaxSim motorSim;
    private final SparkMaxAlternateEncoderSim encoderSim;
    private final DIOSim retractedLSSim;
    private final DIOSim extendedLSSim;

    public DeployIOSim() {
        DCMotor gearbox = DCMotor.getNeo550(1);

        deploySim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        gearbox, kCarriageMass.in(Kilograms), kDrumRadius.in(Meters), kMotorToDeployGearing),
                gearbox,
                kRetractedDistance.in(Meters),
                kExtendedDistance.in(Meters),
                false,
                kRetractedDistance.in(Meters));

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new SparkMaxAlternateEncoderSim(motor);
        retractedLSSim = new DIOSim(retractedLS);
        extendedLSSim = new DIOSim(extendedLS);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        deploySim.setInput(
                motorSim.getAppliedOutput() * PowerSim.getRailVoltage().in(Volts));
        deploySim.update(0.02);
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.of(motorSim.getMotorCurrent()));
    }

    @Override
    public void updateIOSim() {
        LinearVelocity carriageVelocity = MetersPerSecond.of(deploySim.getVelocityMetersPerSecond());
        AngularVelocity drumVelocity =
                RadiansPerSecond.of(carriageVelocity.in(MetersPerSecond) / kDrumRadius.in(Meters));
        AngularVelocity motorVelocity = drumVelocity.times(kMotorToDeployGearing);

        motorSim.iterate(motorVelocity.in(RPM), PowerSim.getRailVoltage().in(Volts), 0.02);

        Distance deployDistance = Meters.of(deploySim.getPositionMeters());
        Angle drumRotations = Rotations.of(deployDistance.in(Meters) / (2 * Math.PI * kDrumRadius.in(Meters)));
        Angle motorRotations = drumRotations.times(kMotorToDeployGearing);

        encoderSim.setPosition(motorRotations.in(Rotations));
        retractedLSSim.setValue(deploySim.hasHitLowerLimit());
        extendedLSSim.setValue(deploySim.hasHitUpperLimit());

        SimulationManager.getInstance().setIntakeExtended(deploySim.hasHitUpperLimit());
    }
}
