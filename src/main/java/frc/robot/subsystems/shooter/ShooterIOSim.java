package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.PowerSim;

public class ShooterIOSim implements ShooterIO {

    private final SparkMax motor;
    private final SparkMaxSim motorSim;
    private final SparkMaxConfig config;

    private final SparkRelativeEncoderSim encoderSim;

    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;

    private AngularVelocity targetVelocity = RPM.zero();

    public ShooterIOSim() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast).inverted(false);

        motor = new SparkMax(kShooterMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));

        encoderSim = motorSim.getRelativeEncoderSim();

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kVelocityTolerance.in(RPM));

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        updateSimulation();

        inputs.velocity = getVelocity();
        inputs.targetVelocity = targetVelocity;
        inputs.appliedVolts = Volts.of(motorSim.getAppliedOutput() * PowerSim.getRailVoltage());
        inputs.outputCurrent = Amps.of(motorSim.getMotorCurrent());
    }

    @Override
    public void updateControl() {
        double pidVolts = pid.calculate(getVelocity().in(RPM), targetVelocity.in(RPM));
        double ffVolts = feedforward.calculate(targetVelocity.in(RPM));

        double volts = pidVolts + ffVolts;
        volts = MathUtil.clamp(volts, -12.0, 12.0);

        motor.setVoltage(volts);
    }

    public void updateSimulation() {
        PowerSim.addCurrentDraw(motorSim.getMotorCurrent());
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(encoderSim.getVelocity());
    }

    @Override
    public boolean atSpeed() {
        return pid.atSetpoint();
    }

    @Override
    public void stop() {
        targetVelocity = RPM.zero();
        pid.reset();
        motor.stopMotor();
    }
}
