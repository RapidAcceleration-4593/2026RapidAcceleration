package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    public DeploySubsystem(DeployIO io) {
        this.io = io;
        this.inputs = new DeployInputsAutoLogged();

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("DeployRoot", 0.5, 0.5);
        deploy = root.append(new LoggedMechanismLigament2d("Deploy", Inches.of(12), Degrees.zero()));

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(kDistanceTolerance.in(Inches));

        Trigger inLimitSwitchTrigger = new Trigger(() -> inputs.inLimitSwitch);
        inLimitSwitchTrigger.onTrue(runOnce(() -> {
            controller.reset();
            controller.setSetpoint(kMinimumDistance.in(Inches));
            io.resetEncoder();
        }));

        Trigger outLimitSwitchTrigger = new Trigger(() -> inputs.outLimitSwitch);
        outLimitSwitchTrigger.onTrue(runOnce(() -> {
            controller.reset();
            controller.setSetpoint(kMaximumDistance.in(Inches));
        }));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Deploy", inputs);

        deploy.setLength(inputs.distance);
        Logger.recordOutput("Mechanisms/Deploy", mechanism);
    }

    public Command goToDistanceCommand(Distance distance) {
        return runOnce(() -> controller.setSetpoint(
                        MathUtil.clamp(distance.in(Inches), kMinimumDistance.in(Inches), kMaximumDistance.in(Inches))))
                .andThen(run(() -> {
                    double output = controller.calculate(inputs.distance.in(Inches));
                    output = MathUtil.clamp(output, -12.0, 12.0);

                    io.setVoltage(Volts.of(output));
                }))
                .until(() -> shouldStopDriving())
                .finallyDo(() -> {
                    stop();
                    controller.setSetpoint(inputs.distance.in(Inches));
                });
    }

    private boolean shouldStopDriving() {
        if (inputs.inLimitSwitch || inputs.outLimitSwitch) return true;
        return controller.atSetpoint();
    }

    public Distance getCurrentDistance() {
        return inputs.distance;
    }

    @AutoLogOutput(key = "Deploy/TargetDistance")
    public Distance getTargetDistance() {
        return Inches.of(controller.getSetpoint());
    }

    @AutoLogOutput(key = "Deploy/AtTargetDistance")
    public boolean atTargetDistance() {
        return controller.atSetpoint();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    private void stop() {
        controller.reset();
        io.stop();
    }
}
