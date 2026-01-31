package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DeployIOReal implements DeployIO {

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;
    protected final DigitalInput inLimitSwitch;
    protected final DigitalInput outLimitSwitch;

    private final SparkClosedLoopController controller;
    private Distance targetDistance = Inches.zero();

    public DeployIOReal() {
        motor = new SparkMax(kDeployMotorID, MotorType.kBrushless);
        encoder = motor.getAlternateEncoder();

        inLimitSwitch = new DigitalInput(kInLimitSwitchChannel);
        outLimitSwitch = new DigitalInput(kOutLimitSwitchChannel);

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(IdleMode.kCoast)
                .inverted(false)
                .apply(new ClosedLoopConfig()
                        .pid(kP, kI, kD)
						.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                        .apply(new MAXMotionConfig()
                                .cruiseVelocity(0)
                                .maxAcceleration(0)
                                .allowedProfileError(0)));
		config.encoder.countsPerRevolution(kCountsPerRotation).positionConversionFactor(kInchesConversionFactor);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();

        Trigger inLimitSwitchTrigger = new Trigger(inLimitSwitch::get);
        inLimitSwitchTrigger.onTrue(Commands.runOnce(() -> {
			controller.setIAccum(0);
            encoder.setPosition(kRetractedDistance.in(Inches));
			controller.setSetpoint(encoder.getPosition(), ControlType.kMAXMotionPositionControl);
        }));

        Trigger outLimitSwitchTrigger = new Trigger(outLimitSwitch::get);
        outLimitSwitchTrigger.onTrue(Commands.runOnce(() -> {
			controller.setIAccum(0);
			encoder.setPosition(kExtendedDistance.in(Inches));
			controller.setSetpoint(encoder.getPosition(), ControlType.kMAXMotionPositionControl);
        }));
    }

    @Override
    public void updateInputs(DeployInputs inputs) {
        inputs.distance = Inches.of(encoder.getPosition());
        inputs.targetDistance = targetDistance;
        inputs.atTargetDistance = controller.isAtSetpoint();

        inputs.inLimitSwitch = inLimitSwitch.get();
        inputs.outLimitSwitch = outLimitSwitch.get();

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    public void setPosition(Distance distance) {
        targetDistance = distance;
        controller.setSetpoint(distance.in(Inches), ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
