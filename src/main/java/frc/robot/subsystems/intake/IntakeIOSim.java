package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {

    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeConfig;

    private final SparkMax deployMotor;
    private final SparkMaxConfig deployConfig;

    private final IntakeSimulation intakeSim;

    public IntakeIOSim(AbstractDriveTrainSimulation drivetrain) {
        intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake).inverted(false);

        deployConfig = new SparkMaxConfig();
        deployConfig.idleMode(IdleMode.kCoast).inverted(false);

        intakeMotor = new SparkMax(kIntakeMotorID, MotorType.kBrushless);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deployMotor = new SparkMax(kDeployMotorID, MotorType.kBrushless);
        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeSim = IntakeSimulation.OverTheBumperIntake(
                "Fuel", drivetrain, Inches.of(30.0), Inches.of(12), IntakeSimulation.IntakeSide.FRONT, kMaxCapacity);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.isIntaking = isIntaking();

        inputs.isDeployed = false;
        inputs.isRetracted = false;

        inputs.intakeVolts = Volts.of(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
        inputs.deployVolts = Volts.of(deployMotor.getAppliedOutput() * deployMotor.getBusVoltage());

        inputs.intakeCurrent = Amps.of(intakeMotor.getOutputCurrent());
        inputs.deployCurrent = Amps.of(deployMotor.getOutputCurrent());
    }

    @Override
    public void setIntakeSpeed(double speed) {
        double volts = speed * intakeMotor.getBusVoltage();
        intakeMotor.setVoltage(volts);
        intakeSim.startIntake();
    }

    @Override
    public void stopIntake() {
        intakeMotor.stopMotor();
        intakeSim.stopIntake();
    }

    @Override
    public void setDeploySpeed(double speed) {
        double volts = speed * deployMotor.getBusVoltage();
        deployMotor.setVoltage(volts);
    }

    @Override
    public void stopDeploy() {
        deployMotor.stopMotor();
    }

    public boolean isIntaking() {
        return Math.abs(intakeMotor.get()) > 0;
    }
}
