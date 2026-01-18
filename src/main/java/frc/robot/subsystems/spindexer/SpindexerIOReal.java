package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SpindexerIOReal implements SpindexerIO {

    private final SparkMax spindexer;
    private final SparkMax feeder;

    private final SparkMaxConfig config;

    public SpindexerIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(false);

        spindexer = new SparkMax(kSpindexerMotorID, MotorType.kBrushless);
        spindexer.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feeder = new SparkMax(kFeederMotorID, MotorType.kBrushless);
        feeder.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(SpindexerInputs inputs) {
        inputs.spindexerVelocity = RPM.of(spindexer.getEncoder().getVelocity());
        inputs.feederVelocity = RPM.of(feeder.getEncoder().getVelocity());

        inputs.spindexerVolts = Volts.of(spindexer.getAppliedOutput() * spindexer.getBusVoltage());
        inputs.feederVolts = Volts.of(feeder.getAppliedOutput() * feeder.getBusVoltage());

        inputs.spindexerCurrent = Amps.of(spindexer.getOutputCurrent());
        inputs.feederCurrent = Amps.of(feeder.getOutputCurrent());
    }

    @Override
    public void setSpindexerSpeed(double speed) {
        spindexer.set(speed);
    }

    @Override
    public void stopSpindexer() {
        spindexer.stopMotor();
    }

    @Override
    public void setFeederSpeed(double speed) {
        feeder.set(speed);
    }

    @Override
    public void stopFeeder() {
        feeder.stopMotor();
    }
}
