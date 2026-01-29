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

    protected final SparkMax motor;
    protected final Encoder encoder;

    public ClimberIOReal() {
        SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        motor = new SparkMax(kClimberMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kClimberEncoderChannelA, kClimberEncoderChannelB);
        encoder.setDistancePerPulse(kInchesPerPulse);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.distance = Inches.of(encoder.getDistance());

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
