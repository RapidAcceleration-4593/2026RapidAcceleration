package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOReal implements IntakeIO {

    protected final SparkMax motor;

    private final SparkMaxConfig intakeConfig;

    public IntakeIOReal() {
        intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake).inverted(false).voltageCompensation(12.0);

        motor = new SparkMax(kMotorID, MotorType.kBrushless);
        motor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.isIntaking = isIntaking();

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void setIntakeSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void stopIntake() {
        motor.stopMotor();
    }

    public boolean isIntaking() {
        return Math.abs(motor.get()) > 0;
    }
}
