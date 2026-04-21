package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {

    protected final SparkMax motor;

    public IntakeIOReal() {
        SparkBaseConfig intakeConfig = new SparkMaxConfig()
                .inverted(kInvertMotor)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);

        motor = new SparkMax(kMotorID, MotorType.kBrushless);
        motor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
        inputs.intakeVelocity = RPM.of(motor.getEncoder().getVelocity());
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
