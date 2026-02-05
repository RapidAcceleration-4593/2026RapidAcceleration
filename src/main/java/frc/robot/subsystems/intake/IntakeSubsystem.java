package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kIntakeVolts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final IntakeInputsAutoLogged inputs;
    private final IntakeIO io;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Command", this.getCurrentCommand().getName());
        } else {
            Logger.recordOutput("Command", "none");
        }
    }

    /**
     * Constructs a command to run the intake motor.
     *
     * @return A command to run the intake motor and stop when complete.
     */
    public Command runCommand() {
        return startEnd(() -> io.setVoltage(kIntakeVolts), io::stop);
    }

    /**
     * Constructs a command to stop the intake motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }
}
