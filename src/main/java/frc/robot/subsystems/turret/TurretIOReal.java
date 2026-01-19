package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;

public class TurretIOReal implements TurretIO {

    private final SparkMax motor;
    private final Encoder encoder;
    private final SparkMaxConfig config;

    private final PIDController pid;

    private Angle targetAngle = kInitialAngle;

    public TurretIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(false);

        motor = new SparkMax(kMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kEncoderChannelA, kEncoderChannelB);
        encoder.setDistancePerPulse(kDegreesPerPulse);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kMaximumTolerance.in(Degrees));
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.atAngle = atAngle();
        inputs.angle = getAngle();
        inputs.targetAngle = targetAngle;
        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void updateControl() {
        double current = getAngle().in(Degrees);
        double target = targetAngle.in(Degrees);

        double volts = pid.calculate(current, target);
        motor.setVoltage(volts);
    }

    @Override
    public void setAngle(Angle angle) {
        targetAngle = angle;
    }

    public Angle getAngle() {
        return Degrees.of(encoder.getDistance());
    }

    public boolean atAngle() {
        return pid.atSetpoint();
    }

    @Override
    public void stop() {
        targetAngle = getAngle();
        pid.reset();
        motor.stopMotor();
    }
}
