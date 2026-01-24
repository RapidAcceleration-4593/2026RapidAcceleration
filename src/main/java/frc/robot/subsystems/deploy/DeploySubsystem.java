package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deploy", inputs);

        deploy.setLength(inputs.distance);
        Logger.recordOutput("Mechanisms/Deploy", mechanism);
    }

    public void updateControl() {
        io.updateControl();
    }

    public void setDistance(Distance distance) {
        io.setDistance(distance);
    }

    public Distance getDistance() {
        return inputs.distance;
    }

    public boolean atDistance() {
        return inputs.atTargetDistance;
    }

    public void stop() {
        io.stop();
    }
}
