package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class ClimberIOSim extends ClimberIOReal implements IPhysicsSim {

    private final ElevatorSim climberSim;
    private final SparkMaxSim motorSim;
    private final SparkMaxAlternateEncoderSim encoderSim;

    public ClimberIOSim() {
        DCMotor gearbox = DCMotor.getVex775Pro(1);

        climberSim = new ElevatorSim(
                gearbox,
                kMotorToClimberGearing,
                kCarriageMass.in(Kilograms),
                kDrumRadius.in(Meters),
                kMinimumDistance.in(Meters),
                kMaximumDistance.in(Meters),
                true,
                kMinimumDistance.in(Meters));

        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = new SparkMaxAlternateEncoderSim(motor);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        climberSim.setInput(motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
        climberSim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(climberSim.getCurrentDrawAmps());
    }

    @Override
    public void updateIOSim() {
        var drumRadPS = climberSim.getVelocityMetersPerSecond() / kDrumRadius.in(Meters);
        AngularVelocity motorAngularVelocity = RadiansPerSecond.of(drumRadPS * kMotorToClimberGearing);
        motorSim.iterate(
                motorAngularVelocity.in(RPM),
                SimulatedBattery.getBatteryVoltage().in(Volts),
                0.02);
        Distance climberDistance = Meters.of(climberSim.getPositionMeters());
        encoderSim.setPosition(climberDistance.in(Inches));
    }
}
