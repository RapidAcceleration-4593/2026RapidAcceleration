package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class HoodIOReal implements HoodIO {

    private final SparkMax motor;
    private final SparkMaxConfig config;

    private final Encoder encoder;
    private final DigitalInput limitswitch;

    private final PIDController pid;

    private double targetAngle;
    private double appliedVolts;

    public HoodIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        motor = new SparkMax(kHoodMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kHoodEncoderChannelA, kHoodEncoderChannelB);
        encoder.setDistancePerPulse(kDegreesPerPulse);

        limitswitch = new DigitalInput(kHoodLimitSwitchChannel);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kToleranceAngle.in(Degrees));
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.angle = getAngle();
        inputs.limitswitch = getLimitSwitch();
        inputs.appliedVolts = appliedVolts;
    }

    @Override
    public void updateControl() {
        if (getLimitSwitch()) {
            encoder.reset();
            pid.reset();

            if (targetAngle <= 0.0) {
                appliedVolts = 0.0;
                motor.setVoltage(0.0);
                return;
            }
        }

        appliedVolts = pid.calculate(getAngle(), targetAngle);
        motor.setVoltage(appliedVolts);
    }

    @Override
    public void setAngle(double degrees) {
        targetAngle = degrees;
    }

    @Override
    public double getAngle() {
        return encoder.getDistance();
    }

    @Override
    public boolean atAngle() {
        return pid.atSetpoint();
    }

    @Override
    public void stop() {
        targetAngle = getAngle();
        appliedVolts = 0.0;
        motor.stopMotor();
    }

    @Override
    public boolean getLimitSwitch() {
        return limitswitch.get();
    }
}
