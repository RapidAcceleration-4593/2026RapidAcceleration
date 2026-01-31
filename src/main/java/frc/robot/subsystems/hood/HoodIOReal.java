package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

public class HoodIOReal implements HoodIO {

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;
    protected final DigitalInput limitswitch;

    private final SparkClosedLoopController controller;

    public HoodIOReal() {
        motor = new SparkMax(kHoodMotorID, MotorType.kBrushless);
        encoder = motor.getAlternateEncoder();

        limitswitch = new DigitalInput(kHoodLimitSwitchChannel);

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .apply(new ClosedLoopConfig()
                        .pid(kP, kI, kD)
						.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                        .apply(new MAXMotionConfig()
                                .cruiseVelocity(0)
                                .maxAcceleration(0)
                                .allowedProfileError(0)
                                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)));
		config.encoder.countsPerRevolution(kCountsPerRotation).positionConversionFactor(kDegreesConversionFactor);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.angle = Degrees.of(encoder.getPosition());
        inputs.targetAngle = Degrees.of(encoder.getPosition());
        inputs.atTargetAngle = controller.isAtSetpoint();

        inputs.limitswitch = limitswitch.get();

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void setPosition(Angle angle) {
        controller.setSetpoint(angle.in(Degrees), ControlType.kMAXMotionPositionControl);
    }

	@Override
	public void resetPosition() {
		controller.setIAccum(0);
		encoder.setPosition(0);
		controller.setSetpoint(encoder.getPosition(), ControlType.kMAXMotionPositionControl);
	}

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
