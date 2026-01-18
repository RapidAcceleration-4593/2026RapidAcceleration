package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOReal implements ShooterIO {

    private final SparkMax motor;
    private final SparkMaxConfig config;

    private final RelativeEncoder encoder;

    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;

    private AngularVelocity targetVelocity = RPM.zero();

    public ShooterIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast).inverted(false);

        motor = new SparkMax(kShooterMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kVelocityTolerance.in(RPM));

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.atSpeed = atSpeed();
        inputs.velocity = getVelocity();
        inputs.targetVelocity = targetVelocity;
        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void updateControl() {
        double pidVolts = pid.calculate(getVelocity().in(RPM), targetVelocity.in(RPM));
        double ffVolts = feedforward.calculate(targetVelocity.in(RPM));

        double volts = pidVolts + ffVolts;
        volts = MathUtil.clamp(volts, -12.0, 12.0);

        motor.setVoltage(volts);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(encoder.getVelocity());
    }

    @Override
    public boolean atSpeed() {
        return pid.atSetpoint();
    }

    @Override
    public void stop() {
        targetVelocity = RPM.of(0.0);
        pid.reset();
        motor.stopMotor();
    }
}
