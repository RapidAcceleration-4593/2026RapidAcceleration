package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavIOReal implements QuestNavIO {

    private final QuestNav questNav;

    public QuestNavIOReal() {
        this.questNav = new QuestNav();
    }

    @Override
    public void updateInputs(QuestNavInputs inputs) {
        questNav.commandPeriodic();

        inputs.connected = questNav.isConnected();
        inputs.tracking = questNav.isTracking();
        inputs.latency = Milliseconds.of(questNav.getLatency());

        questNav.getBatteryPercent().ifPresent(battery -> inputs.batteryPercent = battery);
        questNav.getTrackingLostCounter().ifPresent(counter -> inputs.trackingLostCount = counter);

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        inputs.unreadFrames = new PoseFrameLog[frames.length];
        for (int i = 0; i < frames.length; i++) {
            inputs.unreadFrames[i] =
                    new PoseFrameLog(frames[i].questPose3d(), frames[i].dataTimestamp(), frames[i].isTracking());
        }
    }

    @Override
    public void setPose(Pose3d questPose) {
        questNav.setPose(questPose);
    }
}
