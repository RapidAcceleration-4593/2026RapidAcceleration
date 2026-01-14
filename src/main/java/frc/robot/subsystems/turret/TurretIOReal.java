package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Encoder;

public class TurretIOReal implements TurretIO {

    private final SparkMax motor = kTurretMotor;
    private final Encoder encoder = kTurretEncoder;

    public TurretIOReal() {
        // Converts pulses to meaningful units of degrees.
        encoder.setDistancePerPulse(kDegreesPerPulse);
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.angle = Degrees.of(encoder.getDistance());
        inputs.angularVelocity = DegreesPerSecond.of(encoder.getRate());
    }

    @Override
    public void setMotorSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void resetEncoder() {
        encoder.reset();
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }
}
