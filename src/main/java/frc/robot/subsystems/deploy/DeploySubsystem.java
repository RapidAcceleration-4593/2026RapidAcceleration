package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class DeploySubsystem extends SubsystemBase {

    private final DeployInputsAutoLogged inputs;
    private final DeployIO io;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d deploy;

    public DeploySubsystem(DeployIO io) {
        this.io = io;
        this.inputs = new DeployInputsAutoLogged();

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("DeployRoot", 0.5, 0.5);
        deploy = root.append(new LoggedMechanismLigament2d("Deploy", Inches.of(12), Degrees.zero()));

        Trigger lsTrigger = new Trigger(() -> (inputs.inLimitSwitch || inputs.outLimitSwitch));
        lsTrigger.onTrue(Commands.runOnce(io::resetPosition));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deploy", inputs);

        deploy.setLength(inputs.distance);
        Logger.recordOutput("Mechanisms/Deploy", mechanism);
    }

    public Command goToDistanceCommand(Distance distance) {
        return runOnce(() -> io.setPosition(distance));
    }

    public Distance getCurrentDistance() {
        return inputs.distance;
    }

    public Distance getTargetDistance() {
        return inputs.targetDistance;
    }

    public boolean atTargetDistance() {
        return inputs.atTargetDistance;
    }

    public Command stopCommand() {
        return runOnce(io::stop);
    }
}
