package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class HoodIOReal implements HoodIO {

    private final SparkMax motor;
    private final SparkMaxConfig config;

    private final Encoder encoder;
    private final DigitalInput limitswitch;
    private final Trigger limitswitchTrigger;

    private final PIDController pid;

    private Angle targetAngle = Degrees.zero();

    public HoodIOReal() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(false);

        motor = new SparkMax(kHoodMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kHoodEncoderChannelA, kHoodEncoderChannelB);
        encoder.setDistancePerPulse(kDegreesPerPulse);

        limitswitch = new DigitalInput(kHoodLimitSwitchChannel);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kAngleTolerance.in(Degrees));

        limitswitchTrigger = new Trigger(this::getLimitSwitch);
        limitswitchTrigger.onTrue(Commands.runOnce(() -> {
            encoder.reset();
            pid.reset();
        }));
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.angle = getAngle();
        inputs.targetAngle = targetAngle;
        inputs.limitswitch = getLimitSwitch();
        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void updateControl() {
        if (getLimitSwitch()) {
            targetAngle = Degrees.of(Math.max(0.0, targetAngle.in(Degrees)));
        }

        double currentDeg = getAngle().in(Degrees);
        double targetDeg = targetAngle.in(Degrees);

        double volts = pid.calculate(currentDeg, targetDeg);
        volts = MathUtil.clamp(volts, -12.0, 12.0);

        motor.setVoltage(volts);
    }

    @Override
    public void setAngle(Angle angle) {
        targetAngle = angle;
    }

    @Override
    public Angle getAngle() {
        return Degrees.of(encoder.getDistance());
    }

    @Override
    public boolean atAngle() {
        return pid.atSetpoint();
    }

    @Override
    public void stop() {
        targetAngle = getAngle();
        pid.reset();
        motor.stopMotor();
    }

    @Override
    public boolean getLimitSwitch() {
        return limitswitch.get();
    }
}
