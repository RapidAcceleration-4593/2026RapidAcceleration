package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class HoodIOReal implements HoodIO {

    protected final SparkMax motor;
    protected final Encoder encoder;
    protected final DigitalInput limitswitch;

    public HoodIOReal() {
        SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false);

        motor = new SparkMax(kHoodMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kHoodEncoderChannelA, kHoodEncoderChannelB);
        encoder.setDistancePerPulse(kDegreesPerPulse);

        limitswitch = new DigitalInput(kHoodLimitSwitchChannel);
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.angle = Degrees.of(encoder.getDistance());
        inputs.limitswitch = limitswitch.get();

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
