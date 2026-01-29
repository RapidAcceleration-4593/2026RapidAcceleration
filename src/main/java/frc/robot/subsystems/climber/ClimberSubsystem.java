package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private Distance targetDistance = kMinimumDistance;

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

    public Command updateControl() {
        return run(() -> {
            targetDistance = Inches.of(MathUtil.clamp(
                    targetDistance.in(Inches), kMinimumDistance.in(Inches), kMaximumDistance.in(Inches)));

            double output = controller.calculate(inputs.distance.in(Inches), targetDistance.in(Inches));
            output = MathUtil.clamp(output, -12.0, 12.0);

            io.setVoltage(Volts.of(output));
        });
    }

    public Command setTargetDistance(Distance distance) {
        return runOnce(() -> targetDistance = distance);
    }

    public Distance getCurrentDistance() {
        return inputs.distance;
    }

    public Distance getTargetDistance() {
        return targetDistance;
    }

    public boolean atTargetDistance() {
        return controller.atSetpoint();
    }

    public Command stop() {
        return runOnce(() -> io.stop());
    }
}
