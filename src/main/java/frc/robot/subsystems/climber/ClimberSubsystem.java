package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fLengthMechanism3D;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandLogger;
import frc.robot.util.mechanism.LengthMechanism3D;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberInputsAutoLogged inputs;
    private final ClimberIO io;

    private Distance targetDistance = kMinimumDistance;

    private final LengthMechanism3D climber3D;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        this.inputs = new ClimberInputsAutoLogged();

        climber3D = fLengthMechanism3D.find("Climber");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        targetDistance = inputs.targetDistance;

        climber3D.setLength(inputs.distance);
        CommandLogger.logSubsystemCommand(this);
    }

    public Distance getCurrentDistance() {
        return inputs.distance;
    }

    public Distance getTargetDistance() {
        return targetDistance;
    }

    public boolean atTargetDistance() {
        return inputs.distance.isNear(targetDistance, kDistanceTolerance);
    }

    /**
     * Constructs a command to run the climber at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the motor voltage and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setVoltage(volts), io::stop);
    }

    /**
     * Constructs a command to run the climber to a set distance.
     *
     * @param distance The distance to apply to the open-loop PID control.
     * @return A command to run the motor to a distance and stop when complete.
     */
    public Command goToDistanceCommand(Distance distance) {
        return startEnd(() -> setPosition(distance), io::stop).until(this::atTargetDistance);
    }

    /**
     * Constructs a command to stop the climber motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopMotor() {
        return runOnce(io::stop);
    }

    /**
     * Sets the distance of the closed-loop PID controller.
     *
     * @param distance The distance to set as the climber position.
     */
    private void setPosition(Distance distance) {
        this.targetDistance = distance;
        io.setPosition(distance);
    }
}
