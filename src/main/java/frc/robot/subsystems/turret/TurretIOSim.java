package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;

public class TurretIOSim extends TurretIOReal implements IPhysicsSim {

    private final SingleJointedArmSim turretSim;

    private final SparkMaxSim motorSim;

    public TurretIOSim() {
        DCMotor gearbox = DCMotor.getNeo550(1);

        turretSim = new SingleJointedArmSim(
                gearbox,
                kMotorToTurretGearing,
                kTurretMOI.in(KilogramSquareMeters),
                Units.inchesToMeters(10),
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                false,
                kInitialAngle.in(Radians));

        motorSim = new SparkMaxSim(motor, gearbox);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        turretSim.setInput(motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
        turretSim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(motorSim.getMotorCurrent());
    }

    @Override
    public void updateIOSim() {
        AngularVelocity turretVelocity = RadiansPerSecond.of(turretSim.getVelocityRadPerSec());

        motorSim.iterate(
                turretVelocity.in(DegreesPerSecond) * 60, SimulatedBattery.getBatteryVoltage().in(Volts), 0.02);
    }
}
