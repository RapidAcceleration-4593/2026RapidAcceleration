package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class QuestNavSubsystem extends SubsystemBase {

    private static final Transform3d kRobotToQuest =
            new Transform3d(Inches.zero(), Inches.zero(), Inches.zero(), new Rotation3d());
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

                visionConsumer.accept(robotPose.toPose2d(), frame.timestamp(), kStateSTDDevs);

                Logger.recordOutput("QuestNav/RobotPose", robotPose);
            }
        }
    }

    public void resetPose(Pose3d robotPose) {
        Pose3d questPose = robotPose.transformBy(kRobotToQuest);
        io.setPose(questPose);
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
    }
}
