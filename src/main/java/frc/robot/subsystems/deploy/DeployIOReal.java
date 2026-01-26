package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

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

public class DeployIOReal implements DeployIO {

    protected final SparkMax motor;
    protected final Encoder encoder;
    protected final DigitalInput limitswitch;

    public DeployIOReal() {
        SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kCoast);

        motor = new SparkMax(kDeployMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kEncoderChannelA, kEncoderChannelB);
        encoder.setDistancePerPulse(kDistancePerPulse);

        limitswitch = new DigitalInput(kLimitSwitchChannel);
    }

    @Override
    public void updateInputs(DeployInputs inputs) {
        inputs.distance = Inches.of(encoder.getDistance());
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
