package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fLengthMechanism3D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    }

    public Command goToDistanceCommand(Distance distance) {
        return runOnce(() -> controller.setSetpoint(
                        MathUtil.clamp(distance.in(Inches), kMinimumDistance.in(Inches), kMaximumDistance.in(Inches))))
                .andThen(run(() -> {
                    double output = controller.calculate(inputs.distance.in(Inches));
                    output = MathUtil.clamp(output, -12.0, 12.0);

                    io.setVoltage(Volts.of(output));
                }))
                .until(this::shouldStop)
                .finallyDo(() -> {
                    stop();
                    controller.setSetpoint(inputs.distance.in(Inches));
                });
    }

    private boolean shouldStop() {
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

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    private void stop() {
        io.stop();
    }
}
