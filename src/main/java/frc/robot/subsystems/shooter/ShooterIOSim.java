package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.SimulationManager;

public class ShooterIOSim extends ShooterIOReal implements IPhysicsSim {

    private final SparkMaxSim motorSim;
    private final SparkRelativeEncoderSim encoderSim;
    private final DCMotor gearbox;
    private final FlywheelSim flywheelSim;

    public ShooterIOSim() {
        gearbox = DCMotor.getNEO(1);
        motorSim = new SparkMaxSim(motor, gearbox);
        encoderSim = motorSim.getRelativeEncoderSim();
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        gearbox, kShooterWheelMOI.in(KilogramSquareMeters), kShooterWheelGearing),
                gearbox);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        flywheelSim.setInput(motor.getAppliedOutput() * RobotController.getBatteryVoltage());
        flywheelSim.update(0.02);
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.of(flywheelSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateIOSim() {
        motorSim.iterate(
                flywheelSim.getAngularVelocityRPM(), PowerSim.getRailVoltage().in(Volts), 0.05);
        encoderSim.setVelocity(motorSim.getVelocity());
    }
}
