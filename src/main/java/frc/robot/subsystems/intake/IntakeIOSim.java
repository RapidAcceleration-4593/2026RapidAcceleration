package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.IPhysicsSim;
import frc.robot.util.PowerSim;
import frc.robot.util.SimulationManager;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim extends IntakeIOReal implements IPhysicsSim {

    private final SparkMaxSim motorSim;

    private final IntakeSimulation intakeSim;

    public IntakeIOSim(AbstractDriveTrainSimulation drivetrain) {
        DCMotor gearbox = DCMotor.getNEO(1);

        motorSim = new SparkMaxSim(motor, gearbox);
        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel", drivetrain, Inches.of(30.0), Inches.of(12), IntakeSimulation.IntakeSide.FRONT, kMaxCapacity);

        SimulationManager.getInstance().addSimulatable(this);
    }


    @Override
    public void updatePlantSim() {
        // TODO: Implement.
    }

    @Override
    public void updatePowerSim() {
        PowerSim.addCurrentDraw(Amps.of(motorSim.getMotorCurrent()));
    }

    @Override
    public void updateIOSim() {
        // TODO: Implement.
    }
}
