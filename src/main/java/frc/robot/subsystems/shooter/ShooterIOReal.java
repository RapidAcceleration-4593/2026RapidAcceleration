package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOReal implements ShooterIO {

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;

    private final SparkClosedLoopController controller;
    private AngularVelocity targetVelocity = RPM.zero();

    public ShooterIOReal() {
        motor = new SparkMax(kShooterMotorID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(IdleMode.kCoast)
                .inverted(false)
                .apply(new ClosedLoopConfig()
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(kP, kI, kD)
                        .apply(new FeedForwardConfig().kS(kS).kV(kV).kA(kA))
                        .apply(new MAXMotionConfig()
                                .maxAcceleration(0)
                                .cruiseVelocity(0)
                                .allowedProfileError(0)));

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.velocity = RPM.of(encoder.getVelocity());
        inputs.targetVelocity = targetVelocity;
        inputs.atTargetVelocity =
                Math.abs(targetVelocity.in(RPM) - encoder.getVelocity()) <= kVelocityTolerance.in(RPM);

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
        controller.setSetpoint(velocity.in(RPM), ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void stop() {
        setVelocity(RPM.zero());
    }
}
