package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kIntakeVolts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
    }

    public boolean isIntaking() {
        return inputs.isIntaking;
    }

    /**
     * Constructs a command to run the intake at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the intake motor and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setVoltage(volts));
    }

    /**
     * Constructs a command to run the intake motor.
     *
     * @return A command to run the intake motor and stop when complete.
     */
    public Command runCommand() {
        return runOnce(() -> io.setVoltage(kIntakeVolts)).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
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
