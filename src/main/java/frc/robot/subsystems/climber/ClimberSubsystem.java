package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fLengthMechanism3D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandLogger;
import frc.robot.util.mechanism.LengthMechanism3D;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberInputsAutoLogged inputs;
    private final ClimberIO io;

    private final PIDController controller;

    private final LengthMechanism3D climber3D;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        this.inputs = new ClimberInputsAutoLogged();

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(kDistanceTolerance.in(Inches));
        climber3D = fLengthMechanism3D.find("Climber");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        climber3D.setLength(inputs.distance);
        CommandLogger.logSubsystemCommand(this);
    }

    private boolean shouldStop() {
        // TODO: Implement.
        return false;
    }

    public Distance getCurrentDistance() {
        return inputs.distance;
    }

    @AutoLogOutput(key = "Climber/TargetDistance")
    public Distance getTargetDistance() {
        return Inches.of(controller.getSetpoint());
    }

    @AutoLogOutput(key = "Climber/AtTargetDistance")
    public boolean atTargetDistance() {
        return controller.atSetpoint();
    }

    /**
     * Constructs a command to run the left climber at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the left motor voltage and stop when complete.
     */
    public Command setLeftVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setLeftVoltage(volts), io::stopLeft);
    }

    /**
     * Constructs a command to run the right climber at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the right motor voltage and stop when complete.
     */
    public Command setRightVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setRightVoltage(volts), io::stopRight);
    }

    /**
     * Constructs a command to run the climber to a set distance.
     *
     * @param distance The distance to apply to the open-loop PID control.
     * @return A command to run the motor to a distance and stop when complete.
     */
    public Command goToDistanceCommand(Distance distance) {
        return runOnce(() -> controller.setSetpoint(
                        MathUtil.clamp(distance.in(Inches), kMinimumDistance.in(Inches), kMaximumDistance.in(Inches))))
                .andThen(run(() -> {
                    double output = controller.calculate(inputs.distance.in(Inches));
                    output = MathUtil.clamp(output, -12.0, 12.0);

                    io.setLeftVoltage(Volts.of(output));
                    io.setRightVoltage(Volts.of(output));
                }))
                .until(this::shouldStop)
                .finallyDo(() -> {
                    io.stopLeft();
                    io.stopRight();
                    controller.setSetpoint(inputs.distance.in(Inches));
                });
    }

    /**
     * Constructs a command to stop the left climber motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopLeftCommand() {
        return runOnce(io::stopLeft);
    }

    /**
     * Constructs a command to stop the right climber motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopRightCommand() {
        return runOnce(io::stopRight);
    }
}
