package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterInputsAutoLogged inputs;
    private final ShooterIO io;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        this.inputs = new ShooterInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setTargetVelocity(double rpm) {
        io.setTargetVelocity(rpm);
    }

    public void stop() {
        io.stop();
    }

    public boolean atTargetVelocity() {
        return Math.abs(inputs.velocityRPM - inputs.targetRPM) < kVelocityToleranceRPM.in(RPM);
    }

    public double getVelocity() {
        return inputs.velocityRPM;
    }
}
