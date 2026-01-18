package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

    private final FlywheelSim flywheel;

    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;

    private double targetRPM;
    private double appliedVolts;

    public ShooterIOSim() {
        flywheel = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getNEO(1), kShooterMOI.in(KilogramSquareMeters), kShooterGearRatio),
                DCMotor.getNEO(1));

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kVelocityToleranceRPM.in(RPM));

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        flywheel.update(0.02);

        inputs.appliedVolts = appliedVolts;
        inputs.velocityRPM = getVelocity();
        inputs.targetRPM = targetRPM;
    }

    @Override
    public void updateControl() {
        double pidVolts = pid.calculate(getVelocity(), targetRPM);
        double ffVolts = feedforward.calculate(targetRPM);

        appliedVolts = pidVolts + ffVolts;
        flywheel.setInputVoltage(appliedVolts);
    }

    @Override
    public void setVelocity(double rpm) {
        targetRPM = rpm;
    }

    @Override
    public double getVelocity() {
        return flywheel.getAngularVelocityRPM();
    }

    @Override
    public boolean atSpeed() {
        return pid.atSetpoint();
    }

    @Override
    public void stop() {
        targetRPM = 0.0;
        appliedVolts = 0.0;
        flywheel.setInputVoltage(0.0);
    }
}
