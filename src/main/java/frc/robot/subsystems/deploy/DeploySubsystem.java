package frc.robot.subsystems.deploy;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class DeploySubsystem extends SubsystemBase {

    private final DeployInputsAutoLogged inputs;
    private final DeployIO io;

    public DeploySubsystem(DeployIO io) {
        this.io = io;
        this.inputs = new DeployInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deploy", inputs);
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

    public void stopDeploy() {
        io.stopDeploy();
    }
}
