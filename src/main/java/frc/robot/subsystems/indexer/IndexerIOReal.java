package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerIOReal implements IndexerIO {

    protected final SparkMax spindexerMotor;
    protected final SparkMax feederMotor;
    protected final DigitalInput sensor;

    private final SparkClosedLoopController spindexerController;
    private final SparkClosedLoopController feederController;

    public IndexerIOReal() {
        SparkBaseConfig spindexerConfig = new SparkMaxConfig()
                .inverted(kInvertSpindexerMotor)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(20)
                .voltageCompensation(12.0);

        ClosedLoopConfig spindexerCLCConfig = new ClosedLoopConfig().apply(new FeedForwardConfig().kV(kVSpindexer));
        spindexerConfig.apply(spindexerCLCConfig);

        SparkBaseConfig feederConfig = new SparkMaxConfig()
                .inverted(kInvertFeederMotor)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);

        ClosedLoopConfig feederCLCConfig = new ClosedLoopConfig().apply(new FeedForwardConfig().kV(kVFeeder));
        feederConfig.apply(feederCLCConfig);

        spindexerMotor = new SparkMax(kSpindexerMotorID, MotorType.kBrushless);
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotor = new SparkMax(kFeederMotorID, MotorType.kBrushless);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sensor = new DigitalInput(kSensorChannel);

        spindexerController = spindexerMotor.getClosedLoopController();
        feederController = feederMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        inputs.spindexerVelocity = RPM.of(spindexerMotor.getEncoder().getVelocity());
        inputs.feederVelocity = RPM.of(feederMotor.getEncoder().getVelocity());
        inputs.spindexerTargetVelocity = RPM.of(spindexerController.getSetpoint());
        inputs.feederTargetVelocity = RPM.of(feederController.getSetpoint());

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
    public void setSpindexerVelocity(AngularVelocity velocity) {
        spindexerController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    @Override
    public void setFeederVoltage(Voltage volts) {
        feederMotor.setVoltage(volts);
    }

    @Override
    public void setFeederVelocity(AngularVelocity velocity) {
        feederController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
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
