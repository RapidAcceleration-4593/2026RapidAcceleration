package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class ShooterIOSim extends ShooterIOReal implements IPhysicsSim {

    private final TalonFXSimState motorSim;
    private final FlywheelSim flywheelSim;

    public ShooterIOSim() {
        DCMotor gearbox = DCMotor.getKrakenX60(1);

        motorSim = motor.getSimState();
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(gearbox, kShooterMOI.in(KilogramSquareMeters), kShooterGearing),
                gearbox);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        motorSim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage().in(Volts));
        flywheelSim.setInput(motorSim.getMotorVoltage());
        flywheelSim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(flywheelSim.getCurrentDrawAmps());
    }

    @Override
    public void updateIOSim() {
        motorSim.setRotorVelocity(flywheelSim.getAngularVelocity());
    }
}
