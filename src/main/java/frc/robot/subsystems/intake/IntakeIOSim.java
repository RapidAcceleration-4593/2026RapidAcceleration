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
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class IntakeIOSim extends IntakeIOReal implements IPhysicsSim {

    private final SparkMaxSim motorSim;
    private final IntakeSimulation intakeSim;

    private final FlywheelSim flywheelSim;

    public IntakeIOSim(AbstractDriveTrainSimulation drivetrain) {
        DCMotor gearbox = DCMotor.getNEO(1);

        motorSim = new SparkMaxSim(motor, gearbox);

        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel", drivetrain, Inches.of(26.5), Inches.of(11), IntakeSimulation.IntakeSide.FRONT, kMaxCapacity);

        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(gearbox, kIntakeMOI.in(KilogramSquareMeters), kIntakeGearing),
                gearbox);

        SimulationManager.getInstance().addSimulatable(this);
    }

    @Override
    public void updatePlantSim() {
        flywheelSim.setInput(motorSim.getAppliedOutput()
                * SimulatedBattery.getBatteryVoltage().in(Volts));
    }

    @Override
    public Current getCurrentDraw() {
        return Amps.of(flywheelSim.getCurrentDrawAmps());
    }

    @Override
    public void updateIOSim() {
        motorSim.iterate(
                flywheelSim.getAngularVelocityRPM() * kIntakeGearing,
                SimulatedBattery.getBatteryVoltage().in(Volts),
                0.02);
        if (SimulationManager.getInstance().isIntakeExtended()) {
            intakeSim.startIntake();
        } else {
            intakeSim.stopIntake();
        }
    }
}
