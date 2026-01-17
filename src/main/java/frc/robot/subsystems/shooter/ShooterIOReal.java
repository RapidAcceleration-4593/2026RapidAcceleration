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

public class ShooterIOReal implements ShooterIO {

    private final SparkMax motor;
    private final SparkMaxConfig config;

    private final RelativeEncoder encoder;
    private final PIDController pid;

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
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        inputs.velocityRPM = getVelocity();
        inputs.targetRPM = targetRPM;
        inputs.appliedVolts = appliedVolts;
    }

    @Override
    public void setTargetVelocity(double rpm) {
        targetRPM = rpm;
        appliedVolts = pid.calculate(getVelocity(), targetRPM);

        setVelocity(appliedVolts);
    }

    @Override
    public void stop() {
        targetRPM = 0.0;
        appliedVolts = 0.0;
        motor.stopMotor();
    }

    @Override
    public void setVelocity(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public boolean atSpeed() {
        return pid.atSetpoint();
    }
}
