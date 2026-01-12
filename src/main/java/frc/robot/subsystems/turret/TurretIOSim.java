package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.TurretHardwareConstants.*;
import static frc.robot.subsystems.turret.TurretConstants.TurretMechanismConstants.*;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Encoder;

public class TurretIOSim implements TurretIO {

    private final SparkMax motor = turretMotor;
    private final Encoder encoder = turretEncoder;

    public TurretIOSim() {
        // Converts pulses to meaningful units of degrees.
        encoder.setDistancePerPulse(degreesPerPulse);
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
