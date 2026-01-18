package frc.robot.subsystems.spindexer;

import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SpindexerIOReal implements SpindexerIO {

    private final SparkMax spindexerMotor;
    private final SparkMax feederMotor;

    private final SparkMaxConfig config;

    public SpindexerIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        spindexerMotor = new SparkMax(kSpindexerMotorID, MotorType.kBrushless);
        spindexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotor = new SparkMax(kFeederMotorID, MotorType.kBrushless);
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(SpindexerInputs inputs) {}

    @Override
    public void setSpindexerSpeed(double speed) {
        spindexerMotor.set(speed);
    }

    @Override
    public void stopSpindexer() {
        spindexerMotor.stopMotor();
    }

    @Override
    public void setFeederSpeed(double speed) {
        feederMotor.set(speed);
    }

    @Override
    public void stopFeeder() {
        feederMotor.stopMotor();
    }
}
