package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberInputsAutoLogged inputs;
    private final ClimberIO io;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d climber;

    private final PIDController controller;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        this.inputs = new ClimberInputsAutoLogged();

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("ClimberRoot", 0.5, 0.5);
        climber = root.append(new LoggedMechanismLigament2d("Climber", Inches.of(12), Degrees.zero()));

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(kDistanceTolerance.in(Inches));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        climber.setLength(inputs.distance);
        Logger.recordOutput("Mechanism/Climber", mechanism);
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
        return Commands.runOnce(() -> io.setLeftVoltage(volts)).finallyDo(io::stopLeft);
    }

	/**
     * Constructs a command to run the right climber at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the right motor voltage and stop when complete.
     */
    public Command setRightVoltageCommand(Voltage volts) {
        return Commands.runOnce(() -> io.setRightVoltage(volts)).finallyDo(io::stopRight);
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
