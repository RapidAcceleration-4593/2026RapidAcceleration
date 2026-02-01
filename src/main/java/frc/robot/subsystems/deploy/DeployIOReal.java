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

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(IdleMode.kCoast)
                .inverted(false)
                .apply(new ClosedLoopConfig()
                        .pid(kP, kI, kD)
                        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                        .apply(new MAXMotionConfig()
                                .cruiseVelocity(kCruiseVelocity.in(RPM))
                                .maxAcceleration(kMaxAcceleration.in(RPM.per(Second)))
                                .allowedProfileError(kDistanceTolerance.in(Inches))));
        config.encoder.countsPerRevolution(kCountsPerRotation).positionConversionFactor(kInchesConversionFactor);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(DeployInputs inputs) {
        inputs.distance = Inches.of(encoder.getPosition());
        inputs.targetDistance = Inches.of(controller.getSetpoint());
        inputs.atTargetDistance = controller.isAtSetpoint();

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

        controller.setSetpoint(encoder.getPosition(), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    private boolean isAtRetracted() {
        return retractedLS.get() ^ kInvertInLS;
    }

    private boolean isAtExtended() {
        return extendedLS.get() & kInvertOutLS;
    }
}
