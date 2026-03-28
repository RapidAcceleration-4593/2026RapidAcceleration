package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim extends IntakeIOReal implements IPhysicsSim {

    private final SparkMaxSim motorSim;

    private final FlywheelSim flywheelSim;

    public IntakeIOSim() {
        DCMotor gearbox = DCMotor.getNEO(1);

        motorSim = new SparkMaxSim(motor, gearbox);
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(gearbox, kIntakeMOI.in(KilogramSquareMeters), 1 / kMotorToIntakeGearing),
                gearbox);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        flywheelSim.setInput(motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
		flywheelSim.update(0.02);
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(flywheelSim.getCurrentDrawAmps());
    }

    @Override
    public void updateIOSim() {
        motorSim.iterate(
                flywheelSim.getAngularVelocityRPM() * kMotorToIntakeGearing,
                SimulatedBattery.getBatteryVoltage().in(Volts),
                0.02);
		
		Logger.recordOutput("FlywheelRPM", flywheelSim.getAngularVelocityRPM());
        if (flywheelSim.getAngularVelocityRPM() > kMinimumIntakeRPM) {
            SimulationManager.getInstance().setIntakeSpinning(true);
        } else {
            SimulationManager.getInstance().setIntakeSpinning(false);
        }
		Logger.recordOutput("IntakeSpinning", SimulationManager.getInstance().isIntakeSpinning());
		Logger.recordOutput("IntakeExtended", SimulationManager.getInstance().isIntakeExtended());
    }
}
