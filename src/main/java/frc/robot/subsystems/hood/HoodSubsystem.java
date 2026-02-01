package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Field.*;
import static frc.robot.subsystems.hood.HoodConstants.*;
import static frc.robot.util.mechanism.MechanismFinder.fAngleMechanism3D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.mechanism.AngleMechanism3D;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final HoodInputsAutoLogged inputs;
    private final HoodIO io;

    private final PIDController controller;

    private final AngleMechanism3D hood3d;

    public HoodSubsystem(HoodIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new HoodInputsAutoLogged();
        this.poseSupplier = poseSupplier;

        controller = new PIDController(kP, kI, kD);
        controller.setTolerance(kAngleTolerance.in(Degrees));

        Trigger lsTrigger = new Trigger(() -> inputs.limitswitch);
        lsTrigger.onTrue(Commands.runOnce(() -> {
            controller.reset();
            controller.setSetpoint(kMinimumAngle.in(Degrees));
            io.resetEncoder();
        }));
        hood3d = fAngleMechanism3D.find("Hood");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);

        hood3d.setAngle(inputs.angle);
    }

    public Command goToAngleCommand(Angle angle) {
        return runOnce(() -> controller.setSetpoint(
                        MathUtil.clamp(angle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees))))
                .andThen(run(() -> {
                    double output = controller.calculate(inputs.angle.in(Degrees));
                    output = MathUtil.clamp(output, -12.0, 12.0);

                    io.setVoltage(Volts.of(output));
                }))
                .until(this::shouldStop)
                .finallyDo(() -> {
                    stop();
                    controller.setSetpoint(inputs.angle.in(Degrees));
                });
    }

    private boolean shouldStop() {
        if (inputs.limitswitch) return true;
        return controller.atSetpoint();
    }

    public Angle getCurrentAngle() {
        return inputs.angle;
    }

    @AutoLogOutput(key = "Hood/TargetAngle")
    public Angle getTargetAngle() {
        return Degrees.of(controller.getSetpoint());
    }

    @AutoLogOutput(key = "Hood/AtTargetAngle")
    public boolean atTargetAngle() {
        return controller.atSetpoint();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    private void stop() {
        controller.reset();
        io.stop();
    }

    public Command setAngleToHubCommand() {
        return goToAngleCommand(calculateHubAngle());
    }

    private Angle calculateHubAngle() {
        Pose2d targetPose =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? kBlueHubPose : kRedHubPose;
        Pose2d shooterPose = poseSupplier.get().plus(kPhysicalOffset);

        Distance distance = Meters.of(shooterPose.getTranslation().getDistance(targetPose.getTranslation()));

        return Degrees.of(8.0 * distance.in(Meters));
    }
}
