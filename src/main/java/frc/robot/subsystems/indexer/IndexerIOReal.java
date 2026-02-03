package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOReal implements IndexerIO {

    protected final SparkMax spindexerMotor;
    protected final SparkMax feederMotor;

    public IndexerIOReal() {
        SparkBaseConfig spindexerConfig =
                new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(false);
        SparkBaseConfig feederConfig =
                new SparkMaxConfig().idleMode(IdleMode.kCoast).inverted(false);

        spindexerMotor = new SparkMax(kSpindexerMotorID, MotorType.kBrushless);
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotor = new SparkMax(kFeederMotorID, MotorType.kBrushless);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public void setSpindexerVoltage(Voltage volts) {
        spindexerMotor.setVoltage(volts);
    }

    @Override
    public void setFeederVoltage(Voltage volts) {
        feederMotor.setVoltage(volts);
    }

    @Override
    public void stopSpindexer() {
        spindexerMotor.stopMotor();
    }

    @Override
    public void stopFeeder() {
        feederMotor.stopMotor();
    }
}
