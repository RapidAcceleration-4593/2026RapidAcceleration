package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

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
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;

public class DeployIOReal implements DeployIO {

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;
    protected final DigitalInput retractedLS;
    protected final DigitalInput extendedLS;

    private final SparkClosedLoopController controller;

    public DeployIOReal() {
        motor = new SparkMax(kDeployMotorID, MotorType.kBrushless);
        encoder = motor.getAlternateEncoder();

        retractedLS = new DigitalInput(kRetractedLSChannel);
        extendedLS = new DigitalInput(kExtendedLSChannel);

        SparkBaseConfig baseConfig =
                new SparkMaxConfig().inverted(false).idleMode(IdleMode.kCoast).smartCurrentLimit(60);

        AlternateEncoderConfig altEncoderConfig = new AlternateEncoderConfig()
                .inverted(false)
                .countsPerRevolution(kCountsPerRotation)
                .positionConversionFactor(kPositionConversionFactor)
                .velocityConversionFactor(kVelocityConversionFactor);

        ClosedLoopConfig controlConfig = new ClosedLoopConfig()
                .pid(kP, kI, kD)
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .apply(new MAXMotionConfig()
                        .cruiseVelocity(kCruiseVelocity.in(InchesPerSecond))
                        .maxAcceleration(kMaxAcceleration.in(InchesPerSecondPerSecond)));

        SparkMaxConfig config = new SparkMaxConfig();
        config.apply(baseConfig);
        config.apply(altEncoderConfig);
        config.apply(controlConfig);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(DeployInputs inputs) {
        inputs.distance = Inches.of(encoder.getPosition());
        inputs.targetDistance = Inches.of(controller.getSetpoint());

        inputs.inLimitSwitch = isAtRetracted();
        inputs.outLimitSwitch = isAtExtended();

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void setPosition(Distance distance) {
        controller.setSetpoint(distance.in(Inches), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void resetPosition() {
        controller.setIAccum(0);

        if (isAtRetracted()) {
            encoder.setPosition(kRetractedDistance.in(Inches));
        } else if (isAtExtended()) {
            encoder.setPosition(kExtendedDistance.in(Inches));
        }

        setPosition(Inches.of(encoder.getPosition()));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    private boolean isAtRetracted() {
        return retractedLS.get() ^ kInvertInLS;
    }

    private boolean isAtExtended() {
        return extendedLS.get() ^ kInvertOutLS;
    }
}
