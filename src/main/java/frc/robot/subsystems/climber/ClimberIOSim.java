package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.SimulationManager;

public class ClimberIOSim extends ClimberIOReal implements IPhysicsSim {

    private final ElevatorSim climberSim;

    private final SparkMaxSim leftMotorSim;
    private final SparkMaxSim rightMotorSim;
    private final EncoderSim encoderSim;

    public ClimberIOSim() {
        DCMotor gearbox = DCMotor.getNEO(2);

        climberSim = new ElevatorSim(
                gearbox,
                kMotorToClimberGearing,
                kCarriageMass.in(Kilograms),
                kDrumRadius.in(Meters),
                kMinimumDistance.in(Meters),
                kMaximumDistance.in(Meters),
                true,
                kMinimumDistance.in(Meters));

        leftMotorSim = new SparkMaxSim(leftMotor, gearbox);
        rightMotorSim = new SparkMaxSim(rightMotor, gearbox);
        encoderSim = new EncoderSim(encoder);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
		// TODO: Seperate left & right climber simulations.
		double leftClimberInput = leftMotor.getAppliedOutput() * PowerSim.getRailVoltage().in(Volts);
		double rightClimberInput = leftMotor.getAppliedOutput() * PowerSim.getRailVoltage().in(Volts);
        climberSim.setInput(leftClimberInput + rightClimberInput);
        climberSim.update(0.2);
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.of(climberSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateIOSim() {
        var drumRadPS = climberSim.getVelocityMetersPerSecond() / kDrumRadius.in(Meters);
        AngularVelocity motorAngularVelocity = RadiansPerSecond.of(drumRadPS * kMotorToClimberGearing);
        leftMotorSim.iterate(motorAngularVelocity.in(RPM), PowerSim.getRailVoltage().in(Volts), 0.02);
        rightMotorSim.iterate(motorAngularVelocity.in(RPM), PowerSim.getRailVoltage().in(Volts), 0.02);
        encoderSim.setDistance(climberSim.getPositionMeters());
    }
}
