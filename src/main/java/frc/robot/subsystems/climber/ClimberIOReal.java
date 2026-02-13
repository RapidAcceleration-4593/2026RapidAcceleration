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
import edu.wpi.first.wpilibj.Encoder;

public class ClimberIOReal implements ClimberIO {

    protected final SparkMax leftMotor;
    protected final SparkMax rightMotor;
    protected final Encoder encoder;

    public ClimberIOReal() {
        SparkBaseConfig leftConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);
        SparkBaseConfig rightConfig = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);

        leftMotor = new SparkMax(kLeftClimberMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(kRightClimberMotorID, MotorType.kBrushless);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kClimberEncoderChannelA, kClimberEncoderChannelB);
        encoder.setDistancePerPulse(kInchesPerPulse);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.distance = Inches.of(encoder.getDistance());

        inputs.leftAppliedVolts = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
        inputs.leftOutputCurrent = Amps.of(leftMotor.getOutputCurrent());

        inputs.rightAppliedVolts = Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
        inputs.rightOutputCurrent = Amps.of(rightMotor.getOutputCurrent());
    }

    @Override
    public void setLeftVoltage(Voltage volts) {
        leftMotor.setVoltage(volts);
    }

    @Override
    public void setRightVoltage(Voltage volts) {
        rightMotor.setVoltage(volts);
    }

    @Override
    public void stopLeft() {
        leftMotor.stopMotor();
    }

    @Override
    public void stopRight() {
        rightMotor.stopMotor();
    }
}
