package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.ExtraUnits.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOReal implements ShooterIO {

    protected final SparkMax motor;
    protected final RelativeEncoder encoder;

    private final SparkClosedLoopController controller;
    private final SparkMaxConfig config;

    private AngularVelocity targetVelocity = RPM.zero();

    public ShooterIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast)
                .inverted(false)
                .apply(new ClosedLoopConfig()
                        .pid(kP, kI, kD)
                        .apply(new FeedForwardConfig().kS(kS).kV(kV).kA(kA))
                        .apply(new MAXMotionConfig().maxAcceleration(kMaxAcceleration.in(RPMPerSecond))));

        motor = new SparkMax(kShooterMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
        controller = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.velocity = getVelocity();
        inputs.targetVelocity = targetVelocity;
        inputs.atTargetVelocity = atTargetVelocity();
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
        setVelocity(RPM.of(0));
    }

    private AngularVelocity getVelocity() {
        return RPM.of(encoder.getVelocity());
    }

    private boolean atTargetVelocity() {
        return controller.isAtSetpoint();
    }
}
