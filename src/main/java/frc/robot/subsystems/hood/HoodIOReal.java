package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class HoodIOReal implements HoodIO {

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;
    protected final DigitalInput limitswitch;

    private final SparkClosedLoopController controller;

    public HoodIOReal() {
        motor = new SparkMax(kMotorID, MotorType.kBrushless);
        encoder = motor.getAlternateEncoder();
        limitswitch = new DigitalInput(kLSChannel);

        SparkBaseConfig baseConfig = new SparkMaxConfig()
                .inverted(kInvertMotor)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .voltageCompensation(12.0);

        AlternateEncoderConfig altEncoderConfig = new AlternateEncoderConfig()
                .inverted(kInvertEncoder)
                .countsPerRevolution(kCountsPerRotation)
                .positionConversionFactor(kPositionConversionFactor)
                .velocityConversionFactor(kVelocityConversionFactor);

        ClosedLoopConfig controlConfig = new ClosedLoopConfig()
                .pid(kP, kI, kD)
                .apply(new FeedForwardConfig().kS(kS))
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);

        SoftLimitConfig limitConfig = new SoftLimitConfig()
                .reverseSoftLimit(kMinimumAngle.in(Degrees))
                .reverseSoftLimitEnabled(false)
                .forwardSoftLimit(kMaximumAngle.in(Degrees))
                .forwardSoftLimitEnabled(true);

        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(baseConfig);
        config.apply(altEncoderConfig);
        config.apply(controlConfig);
        config.apply(limitConfig);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();

        resetPosition();
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.angle = Degrees.of(encoder.getPosition());
        inputs.targetAngle = Degrees.of(controller.getSetpoint());

        inputs.bottomLS = limitswitch.get() ^ kInvertLS;

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void setPosition(Angle angle) {
        controller.setSetpoint(angle.in(Degrees), ControlType.kPosition);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void resetPosition() {
        encoder.setPosition(kMinimumAngle.in(Degrees));
        controller.setSetpoint(kMinimumAngle.in(Degrees), ControlType.kPosition);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
