package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

    private final FlywheelSim flywheel;
    private final PIDController pid;

    private double targetRPM;
    private double appliedVolts;

    public ShooterIOSim() {
        flywheel = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getNEO(1), kShooterMOI.in(KilogramSquareMeters), kShooterGearRatio),
                DCMotor.getNEO(1));

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kVelocityToleranceRPM.in(RPM));
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        flywheel.update(0.02);

        inputs.velocityRPM = getVelocity();
        inputs.appliedVolts = appliedVolts;
        inputs.targetRPM = targetRPM;
    }

    @Override
    public void setTargetVelocity(double rpm) {
        targetRPM = rpm;
        appliedVolts = pid.calculate(targetRPM);

        setVelocity(appliedVolts);
    }

    @Override
    public void stop() {
        targetRPM = 0.0;
        appliedVolts = 0.0;
        setVelocity(0.0);;
    }

    @Override
    public void setVelocity(double voltage) {
        flywheel.setInputVoltage(voltage);
    }

    @Override
    public double getVelocity() {
        return flywheel.getAngularVelocityRPM();
    }

    @Override
    public boolean atSpeed() {
        return pid.atSetpoint();
    }
}
