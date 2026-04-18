package frc.robot.subsystems.vision.quest;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class QuestNavIOSim implements QuestNavIO {
    Supplier<Pose2d> robotPoseSupplier;
    Transform3d zeroingOffset = QuestNavSubsystem.kRobotToQuest;

    public QuestNavIOSim(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void updateInputs(QuestNavInputs inputs) {
        inputs.batteryPercent = 100;
        inputs.latency = Seconds.zero();
        inputs.connected = true;
        inputs.tracking = true;
        inputs.trackingLostCount = 0;

        Pose3d questPose = new Pose3d(robotPoseSupplier.get()).transformBy(zeroingOffset);

        inputs.unreadFrames = new PoseFrameLog[] {new PoseFrameLog(questPose, Timer.getFPGATimestamp(), true)};
    }

    public void setPose(Pose3d questPose) {
        zeroingOffset = new Transform3d(new Pose3d(robotPoseSupplier.get()), questPose);
    }
}
