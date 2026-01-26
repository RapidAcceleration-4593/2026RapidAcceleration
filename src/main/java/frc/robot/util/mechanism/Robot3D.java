package frc.robot.util.mechanism;

import edu.wpi.first.math.geometry.Pose3d;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class Robot3D {
    private static Robot3D instance;
    private Pose3d robot = Pose3d.kZero;
    private List<Node3D> mechanisms = new ArrayList<Node3D>();
    private List<Node3D> rootMechanisms = new ArrayList<Node3D>();
    private List<Node3D> loggedMechanisms = new ArrayList<Node3D>();

    private Robot3D() {
        AutoLogOutputManager.addObject(this);
        createMechanisms();
    }

    private void createMechanisms() {

    }

    private void addMechanism(Node3D mechanism) {
        addMechanism(mechanism, true);
    }

    private void addMechanism(Node3D mechanism, boolean logged) {
        mechanisms.add(mechanism);
        if (logged) {
            loggedMechanisms.add(mechanism);
        }
        if (!mechanism.hasParent()) {
            rootMechanisms.add(mechanism);
        }
    }

    public LinearMechanism3D getLinearMechanism(String name) {
        for (Node3D mech : mechanisms) {
            if (mech.getName() == name && mech instanceof LinearMechanism3D) {
                return (LinearMechanism3D)mech;
            }
        }
        throw new IllegalArgumentException("No LinearMechanism3D with name '"+name+"' is on this Robot3D!");
    }

    public LinearMechanism3D getAngularMechanism(String name) {
        for (Node3D mech : mechanisms) {
            if (mech.getName() == name && mech instanceof LinearMechanism3D) {
                return (LinearMechanism3D)mech;
            }
        }
        throw new IllegalArgumentException("No LinearMechanism3D with name '"+name+"' is on this Robot3D!");
    }

    public static Robot3D getInstance() {
        if (instance == null) {
            instance = new Robot3D();
        }
        return instance;
    }

    @AutoLogOutput(key = "Mechanisms3D")
    private Pose3d[] log() {
        for (var mech : rootMechanisms) {
            mech.update();
        }
        Pose3d[] mechPoses = new Pose3d[mechanisms.size()];
        for (int i = 0; i < loggedMechanisms.size(); i++) {
            mechPoses[i] = loggedMechanisms.get(i).getGlobalPose();
        }
        return mechPoses;
    }
}
