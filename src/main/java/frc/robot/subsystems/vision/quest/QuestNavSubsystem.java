package frc.robot.subsystems.vision.quest;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConsumer;
import frc.robot.util.FieldUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class QuestNavSubsystem extends SubsystemBase {

    private final LoggedNetworkBoolean questEnabled =
            new LoggedNetworkBoolean("/AccelerationStation/QuestEnabled", true);

    public static final Transform3d kRobotToQuest = new Transform3d(
            Inches.of(-13.0),
            Inches.of(-12.0),
            Inches.of(14.0),
            new Rotation3d(Degrees.of(90), Degrees.zero(), Degrees.of(180)));
    private static final Matrix<N3, N1> kStateSTDDevs = VecBuilder.fill(0.02, 0.02, 0.035);

    private final QuestNavIO io;
    private final QuestNavInputsAutoLogged inputs;
    private final VisionConsumer visionConsumer;

    public QuestNavSubsystem(QuestNavIO io, VisionConsumer visionConsumer) {
        this.io = io;
        this.inputs = new QuestNavInputsAutoLogged();
        this.visionConsumer = visionConsumer;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("QuestNav", inputs);

        for (var frame : inputs.unreadFrames) {
            if (frame.isTracking()) {
                Pose3d robotPose = frame.questPose().transformBy(kRobotToQuest.inverse());
                if (shouldReject(robotPose)) return;
                if (questEnabled.get()) {
                    visionConsumer.accept(robotPose.toPose2d(), frame.timestamp(), kStateSTDDevs);
                }
            }
        }
    }

    public void resetPose(Pose3d robotPose) {
        io.setPose(robotPose.transformBy(kRobotToQuest));
    }

    public boolean shouldReject(Pose3d robotPose) {
        return robotPose.getMeasureX().lte(Meters.zero())
                || robotPose.getMeasureX().gte(FieldUtil.kFieldLength)
                || robotPose.getMeasureY().lte(Meters.zero())
                || robotPose.getMeasureY().gte(FieldUtil.kFieldWidth);
    }

    public Command cancelOnDisconnect(Command command) {
        return run(() -> {
            CommandScheduler.getInstance().schedule(command);
            if (inputs.trackingLostCount > 0 || !inputs.connected || !inputs.tracking) command.cancel();
        });
    }
}
