package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;

public class ShooterIOSim extends ShooterIOReal implements IPhysicsSim {

    // private final SparkMaxSim motorSim;
    // private final SparkRelativeEncoderSim encoderSim;
    private final FlywheelSim flywheelSim;

    public ShooterIOSim() {
        DCMotor gearbox = DCMotor.getKrakenX60(1);

        // motorSim = new SparkMaxSim(motor, gearbox);
        // encoderSim = motorSim.getRelativeEncoderSim();
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(gearbox, kShooterMOI.in(KilogramSquareMeters), kShooterGearing),
                gearbox);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        // flywheelSim.setInput(
        // motor.getAppliedOutput() * SimulatedBattery.getBatteryVoltage().in(Volts));
        // flywheelSim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(flywheelSim.getCurrentDrawAmps());
    }

    @Override
    public void updateIOSim() {
        // motorSim.iterate(
        //         flywheelSim.getAngularVelocityRPM(),
        //         SimulatedBattery.getBatteryVoltage().in(Volts),
        //         0.02);
        // encoderSim.setVelocity(motorSim.getVelocity());
    }
}
