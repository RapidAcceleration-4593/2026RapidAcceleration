package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;

public class ClimberIOReal implements ClimberIO {

    protected final SparkMax motor;
    protected final Encoder encoder;

    private final SparkMaxConfig config;

    private Distance targetDistance = Inches.zero();

    public ClimberIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        motor = new SparkMax(kClimberMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kClimberEncoderChannelA, kClimberEncoderChannelB);
        encoder.setDistancePerPulse(kInchesPerPulse);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.distance = getDistance();
        inputs.targetDistance = targetDistance;
        inputs.atDistance = atDistance();

        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void updateControl() {
        motor.set(0);
    }

    @Override
    public void setDistance(Distance distance) {
        targetDistance = distance;
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    public void resetEncoder() {
        encoder.reset();
    }

    private Distance getDistance() {
        return Inches.of(encoder.getDistance());
    }

    private boolean atDistance() {
        return false; // TODO: implement atDistance logic
    }
}
