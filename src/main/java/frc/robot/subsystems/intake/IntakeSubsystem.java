package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kIntakeVolts;

import edu.wpi.first.units.measure.Voltage;
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
    }

    public Command setVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setVoltage(volts));
    }

    public Command runCommand() {
        return runOnce(() -> io.setVoltage(kIntakeVolts));
    }

    public Command stopCommand() {
        return runOnce(io::stop);
    }
}
