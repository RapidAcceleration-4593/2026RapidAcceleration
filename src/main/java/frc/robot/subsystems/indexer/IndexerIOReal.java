package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IndexerIOReal implements IndexerIO {

    private final SparkMax spindexerMotor;
    private final SparkMax feederMotor;

    private final SparkMaxConfig config;

    public IndexerIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(false);

        spindexerMotor = new SparkMax(kSpindexerMotorID, MotorType.kBrushless);
        spindexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotor = new SparkMax(kFeederMotorID, MotorType.kBrushless);
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        inputs.spindexerVelocity = RPM.of(spindexerMotor.getEncoder().getVelocity());
        inputs.feederVelocity = RPM.of(feederMotor.getEncoder().getVelocity());

        inputs.spindexerCurrent = Amps.of(spindexerMotor.getOutputCurrent());
        inputs.feederCurrent = Amps.of(feederMotor.getOutputCurrent());

        inputs.spindexerVolts = Volts.of(spindexerMotor.getAppliedOutput() * spindexerMotor.getBusVoltage());
        inputs.feederVolts = Volts.of(feederMotor.getAppliedOutput() * feederMotor.getBusVoltage());
    }

    @Override
    public void setSpindexerSpeed(double speed) {
        double volts = speed * spindexerMotor.getBusVoltage();
        spindexerMotor.setVoltage(volts);
    }

    @Override
    public void stopSpindexer() {
        spindexerMotor.stopMotor();
    }

    @Override
    public void setFeederSpeed(double speed) {
        double volts = speed * feederMotor.getBusVoltage();
        feederMotor.setVoltage(volts);
    }

    @Override
    public void stopFeeder() {
        feederMotor.stopMotor();
    }
}
