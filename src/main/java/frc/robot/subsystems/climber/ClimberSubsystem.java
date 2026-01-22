package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
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

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        this.inputs = new ClimberInputsAutoLogged();

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("ClimberRoot", 0.5, 0.5);
        climber = root.append(new LoggedMechanismLigament2d("Climber", Inches.of(12), Degrees.zero()));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.updateControl();
        Logger.processInputs("Climber", inputs);

        climber.setLength(inputs.distance);
        Logger.recordOutput("Mechanism/Climber", mechanism);
    }

    public void setDistance(Distance distance) {
        io.setDistance(distance);
    }

    public Distance getDistance() {
        return inputs.distance;
    }

    public boolean atDistance() {
        return inputs.atDistance;
    }

    public void stop() {
        io.stop();
    }
}
