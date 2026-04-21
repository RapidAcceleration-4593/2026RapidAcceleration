package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.kIntakeVolts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandLogger;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        CommandLogger.logSubsystemCommand(this);
    }

    /**
     * Constructs a command to run the intake at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the motor voltage and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setVoltage(volts), io::stop);
    }

    public Command runCommand() {
        return run(() -> io.setVoltage(kIntakeVolts))
                .until(this::isJamming)
                .andThen(setVoltageCommand(kIntakeVolts.unaryMinus()).withTimeout(0.5))
                .repeatedly()
                .finallyDo(io::stop);
    }

    /**
     * Constructs a command to stop the intake motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }

    private boolean isJamming() {
        Voltage currentVoltage = inputs.appliedVolts;
        AngularVelocity currentVelocity = inputs.intakeVelocity;

        return currentVoltage.abs(Volts) > 6.0 && currentVelocity.abs(RPM) < 120;
    }
}
