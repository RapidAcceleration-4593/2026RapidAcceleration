package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;

public class ShooterIOSim implements ShooterIO {

    private final SparkMax motor = kShooterMotor;

    private final SparkMaxSim motorSim;
    private final DCMotor gearbox;

    public ShooterIOSim() {
        gearbox = DCMotor.getNEO(1);
        motorSim = new SparkMaxSim(kShooterMotor, gearbox);
    }

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
