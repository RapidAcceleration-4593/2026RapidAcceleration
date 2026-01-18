package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

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
        io.updateControl();
		Logger.processInputs("Shooter", inputs);
    }

    public void setVelocity(double rpm) {
        io.setVelocity(rpm);
    }

    public double getVelocity() {
        return inputs.velocityRPM;
    }

    public boolean atVelocity() {
        return io.atSpeed();
    }

    public void stop() {
        io.stop();
    }
}
