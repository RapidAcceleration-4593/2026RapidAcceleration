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
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOReal implements IndexerIO {

    protected final SparkMax spindexerMotor;
    protected final SparkMax feederMotor;

    protected final DigitalInput sensor;

    public IndexerIOReal() {
        SparkBaseConfig spindexerConfig = new SparkMaxConfig()
                .inverted(kInvertSpindexerMotor)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);

        SparkBaseConfig feederConfig = new SparkMaxConfig()
                .inverted(kInvertFeederMotor)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);

        spindexerMotor = new SparkMax(kSpindexerMotorID, MotorType.kBrushless);
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotor = new SparkMax(kFeederMotorID, MotorType.kBrushless);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        sensor = new DigitalInput(kSensorChannel);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        inputs.spindexerVelocity = RPM.of(spindexerMotor.getEncoder().getVelocity());
        inputs.feederVelocity = RPM.of(feederMotor.getEncoder().getVelocity());

        inputs.isFuelDetected = sensor.get() ^ kInvertSensor;

        inputs.spindexerVolts = Volts.of(spindexerMotor.getAppliedOutput() * spindexerMotor.getBusVoltage());
        inputs.feederVolts = Volts.of(feederMotor.getAppliedOutput() * feederMotor.getBusVoltage());

        inputs.spindexerCurrent = Amps.of(spindexerMotor.getOutputCurrent());
        inputs.feederCurrent = Amps.of(feederMotor.getOutputCurrent());
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
