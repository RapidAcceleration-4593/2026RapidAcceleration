package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;

public class TurretIOReal implements TurretIO {

    protected final SparkMax motor;
    protected final Encoder encoder;

    public TurretIOReal() {
        SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false);

        motor = new SparkMax(kTurretMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kTurretEncoderChannelA, kTurretEncoderChannelB);
        encoder.setDistancePerPulse(kDegreesPerPulse);
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.angle = Degrees.of(encoder.getDistance());
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
