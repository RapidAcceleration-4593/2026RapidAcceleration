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
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;

    private final SparkClosedLoopController controller;

    public ShooterIOReal() {
        motor = new SparkMax(kShooterMotorID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkBaseConfig config = new SparkMaxConfig()
                .idleMode(IdleMode.kCoast)
                .inverted(false)
                .smartCurrentLimit(60)
                .voltageCompensation(12.0)
                .apply(new ClosedLoopConfig()
                        .pid(kP, kI, kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .apply(new FeedForwardConfig().sv(kS, kV))
                        .apply(new MAXMotionConfig()
                                .cruiseVelocity(kCruiseVelocity.in(RPM))
                                .maxAcceleration(kMaxAcceleration.in(RPM.per(Second)))));

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.velocity = RPM.of(encoder.getVelocity());
        inputs.targetVelocity = RPM.of(controller.getSetpoint());

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        controller.setSetpoint(velocity.in(RPM), ControlType.kMAXMotionVelocityControl);
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
