package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.PowerSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim extends IntakeIOReal {

    private final SparkMaxSim intakeMotorSim;
    private final SparkMaxSim deployMotorSim;

    private final DCMotor intakeGearbox;
    private final DCMotor deployGearbox;

    private final IntakeSimulation intakeSim;

    public IntakeIOSim(AbstractDriveTrainSimulation drivetrain) {
        intakeGearbox = DCMotor.getNEO(1);
        deployGearbox = DCMotor.getNEO(1);

        intakeMotorSim = new SparkMaxSim(intakeMotor, intakeGearbox);
        deployMotorSim = new SparkMaxSim(deployMotor, deployGearbox);

        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel", drivetrain, Inches.of(30.0), Inches.of(12), IntakeSimulation.IntakeSide.FRONT, kMaxCapacity);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        super.updateInputs(inputs);
        updateSimulation();
    }

    public void updateSimulation() {
        PowerSim.addCurrentDraw(intakeMotorSim.getMotorCurrent());
        PowerSim.addCurrentDraw(deployMotorSim.getMotorCurrent());
    }

    @Override
    public void setIntakeSpeed(double speed) {
        super.setIntakeSpeed(speed);
        intakeSim.startIntake();
    }

    @Override
    public void stopIntake() {
        super.stopIntake();
        intakeSim.stopIntake();
    }

    public boolean isIntaking() {
        return Math.abs(intakeMotor.get()) > 0;
    }
}
