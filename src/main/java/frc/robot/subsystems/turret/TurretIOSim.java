package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.turret.TurretConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.PowerSim;

public class TurretIOSim implements TurretIO {

    private final SparkMax motor;
    private final SparkMaxSim motorSim;
    private final SparkMaxConfig config;

    private final Encoder encoder;
    private final EncoderSim encoderSim;

    private final SingleJointedArmSim turret;
    private final DCMotor gearbox;
    private final PIDController pid;

    private Angle targetAngle = kInitialAngle;

    public TurretIOSim() {
        config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).inverted(false);

        motor = new SparkMax(kMotorID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));

        encoder = new Encoder(kEncoderChannelA, kEncoderChannelB);
        encoder.setDistancePerPulse(kDegreesPerPulse);
        encoderSim = new EncoderSim(encoder);

        gearbox = DCMotor.getNEO(1);
        turret = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(gearbox, kTurretMOI.in(KilogramSquareMeters), kGearRatio),
                gearbox,
                kGearRatio,
                Units.inchesToMeters(10),
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                false,
                kInitialAngle.in(Radians));

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kMaximumTolerance.in(Degrees));
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        updateSimulation();

        inputs.atAngle = atAngle();
        inputs.angle = getAngle();
        inputs.targetAngle = targetAngle;
        inputs.appliedVolts = Volts.of(motor.getAppliedOutput() * PowerSim.getRailVoltage());
        inputs.outputCurrent = Amps.of(motor.getOutputCurrent());
    }

    @Override
    public void updateControl() {
        double current = getAngle().in(Degrees);
        double target = targetAngle.in(Degrees);

        double volts = pid.calculate(current, target);
        motor.setVoltage(volts);
    }

    private void updateSimulation() {
        turret.setInput(motorSim.getAppliedOutput() * PowerSim.getRailVoltage());
        turret.update(0.02);

        motorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(turret.getVelocityRadPerSec()),
                PowerSim.getRailVoltage(),
                0.02);
        encoderSim.setDistance(Units.radiansToDegrees(turret.getAngleRads()));
        encoderSim.setRate(Units.radiansPerSecondToRotationsPerMinute(turret.getVelocityRadPerSec()));

        PowerSim.addCurrentDraw(motorSim.getMotorCurrent());
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
