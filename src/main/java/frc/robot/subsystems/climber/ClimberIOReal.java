package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOReal implements ClimberIO {

    protected final SparkMax motor;

    public ClimberIOReal() {
        motor = new SparkMax(kMotorID, MotorType.kBrushless);

        SparkBaseConfig baseConfig = new SparkMaxConfig()
                .inverted(kInvertMotor)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);

        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(baseConfig);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
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
