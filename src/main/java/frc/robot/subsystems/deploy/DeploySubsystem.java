package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
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

    private final PIDController controller;
    private Distance targetDistance = kMinimumDistance;

    public DeploySubsystem(DeployIO io) {
        this.io = io;
        this.inputs = new DeployInputsAutoLogged();

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("DeployRoot", 0.5, 0.5);
        deploy = root.append(new LoggedMechanismLigament2d("Deploy", Inches.of(12), Degrees.zero()));

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(kDistanceTolerance.in(Inches));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deploy", inputs);

        deploy.setLength(inputs.distance);
        Logger.recordOutput("Mechanisms/Deploy", mechanism);
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

    @AutoLogOutput(key = "Deploy/TargetDistance")
    public Distance getTargetDistance() {
        return targetDistance;
    }

    @AutoLogOutput(key = "Deploy/AtTargetDistance")
    public boolean atTargetDistance() {
        return controller.atSetpoint();
    }

    public Command stop() {
        return runOnce(() -> {
            controller.reset();
            io.stop();
        });
    }
}
