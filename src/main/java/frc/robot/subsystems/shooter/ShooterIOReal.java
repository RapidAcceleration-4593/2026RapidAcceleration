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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterIOReal implements ShooterIO {

    private final SparkMax motor;
    private final SparkMaxConfig config;
    private final RelativeEncoder encoder;

    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;

    private double targetRPM;
    private double appliedVolts;

    public ShooterIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        motor = new SparkMax(0, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kVelocityToleranceRPM.in(RPM));

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.appliedVolts = appliedVolts;
        inputs.velocityRPM = getVelocity();
        inputs.targetRPM = targetRPM;
    }

    @Override
    public void updateControl() {
        double pidVolts = pid.calculate(getVelocity(), targetRPM);
        double ffVolts = feedforward.calculate(targetRPM);

        appliedVolts = pidVolts + ffVolts;
        motor.setVoltage(appliedVolts);
    }

    @Override
    public void setVelocity(double rpm) {
        targetRPM = rpm;
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public boolean atSpeed() {
        return pid.atSetpoint();
    }

    @Override
    public void stop() {
        targetRPM = 0.0;
        appliedVolts = 0.0;
        motor.stopMotor();
    }
}
