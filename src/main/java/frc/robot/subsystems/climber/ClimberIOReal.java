package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOReal implements ClimberIO {

    protected final SparkMax motor;
    protected final SparkAbsoluteEncoder encoder;

    private final SparkClosedLoopController controller;

    public ClimberIOReal() {
        motor = new SparkMax(kMotorID, MotorType.kBrushed);
        encoder = motor.getAbsoluteEncoder();

        SparkBaseConfig baseConfig = new SparkMaxConfig()
                .inverted(kInvertMotor)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0);

        AbsoluteEncoderConfig altEncoderConfig = new AbsoluteEncoderConfig()
                .inverted(kInvertEncoder)
                .zeroOffset(kEncoderOffset.in(Inches))
                .positionConversionFactor(kPositionConversionFactor)
                .velocityConversionFactor(kVelocityConversionFactor);

        ClosedLoopConfig controlConfig =
                new ClosedLoopConfig().pid(kP, kI, kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(baseConfig);
        config.apply(altEncoderConfig);
        config.apply(controlConfig);

        config.softLimit.reverseSoftLimitEnabled(true).reverseSoftLimit(kMinimumDistance.in(Inches));
        config.softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit(kMaximumDistance.in(Inches));

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.distance = Inches.of(encoder.getPosition());
        inputs.targetDistance = Inches.of(controller.getSetpoint());

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void setPosition(Distance distance) {
        controller.setSetpoint(distance.in(Inches), ControlType.kPosition);
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
