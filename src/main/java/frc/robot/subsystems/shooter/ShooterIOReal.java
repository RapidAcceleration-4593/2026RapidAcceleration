package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.spark.SparkMax;

public class ShooterIOReal implements ShooterIO {

    private final SparkMax motor = kShooterMotor;

    @Override
    public void updateInputs(ShooterInputs inputs) {}

    @Override
    public void setMotorSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }
}
