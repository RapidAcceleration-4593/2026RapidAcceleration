package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.PowerSim;

public class HoodIOSim implements HoodIO {

    private final SparkMax motor;
    private final SparkMaxSim motorSim;
    private final SparkMaxConfig config;

    private final Encoder encoder;
    private final EncoderSim encoderSim;
    private final Trigger limitswitchTrigger;

    private final PIDController pid;

    private Angle targetAngle = kMinimumAngle;

    public HoodIOSim() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(false);

        motor = new SparkMax(kHoodMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSim = new SparkMaxSim(motor, DCMotor.getNeo550(1));

        encoder = new Encoder(kHoodEncoderChannelA, kHoodEncoderChannelB);
        encoder.setDistancePerPulse(kDegreesPerPulse);
        encoderSim = new EncoderSim(encoder);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kAngleTolerance.in(Degrees));

        limitswitchTrigger = new Trigger(this::getLimitSwitch);
        limitswitchTrigger.onTrue(Commands.runOnce(() -> {
            encoderSim.setDistance(0.0);
            pid.reset();
        }));
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        updateSimulation();

        inputs.angle = getAngle();
        inputs.atTargetAngle = atAngle();
        inputs.limitswitch = getLimitSwitch();
        inputs.appliedVolts = Volts.of(motorSim.getAppliedOutput() * PowerSim.getRailVoltage());
        inputs.outputCurrent = Amps.of(motorSim.getMotorCurrent());
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

    private void updateSimulation() {
        PowerSim.addCurrentDraw(motorSim.getMotorCurrent());
    }

    @Override
    public void setAngle(Angle angle) {
        targetAngle = angle;
    }

    @Override
    public Angle getAngle() {
        return Degrees.of(encoderSim.getDistance());
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
        return getAngle().in(Degrees) <= kMinimumAngle.in(Degrees);
    }
}
