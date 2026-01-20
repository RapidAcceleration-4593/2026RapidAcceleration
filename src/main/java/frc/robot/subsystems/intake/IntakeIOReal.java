package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO {

    protected final SparkMax intakeMotor;
    protected final SparkMax deployMotor;

    protected final DigitalInput deployedLS;
    protected final DigitalInput retractedLS;

    private final SparkMaxConfig intakeConfig;
    private final SparkMaxConfig deployConfig;

    public IntakeIOReal() {
        intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake).inverted(false).voltageCompensation(12.0);

        deployConfig = new SparkMaxConfig();
        deployConfig.idleMode(IdleMode.kCoast).inverted(false).voltageCompensation(12.0);

        intakeMotor = new SparkMax(kIntakeMotorID, MotorType.kBrushless);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deployMotor = new SparkMax(kDeployMotorID, MotorType.kBrushless);
        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deployedLS = new DigitalInput(kDeployLS);
        retractedLS = new DigitalInput(kRetractLS);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.isIntaking = isIntaking();

        inputs.isDeployed = isDeployed();
        inputs.isRetracted = isRetracted();

        inputs.intakeVolts = Volts.of(intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
        inputs.deployVolts = Volts.of(deployMotor.getAppliedOutput() * deployMotor.getBusVoltage());

        inputs.intakeCurrent = Amps.of(intakeMotor.getOutputCurrent());
        inputs.deployCurrent = Amps.of(deployMotor.getOutputCurrent());
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.setVoltage(speed);
    }

    @Override
    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    @Override
    public void setDeploySpeed(double speed) {
        deployMotor.setVoltage(speed);
    }

    @Override
    public void stopDeploy() {
        deployMotor.stopMotor();
    }

    public boolean isIntaking() {
        return Math.abs(intakeMotor.get()) > 0;
    }

    public boolean isDeployed() {
        return deployedLS.get();
    }

    public boolean isRetracted() {
        return retractedLS.get();
    }
}
