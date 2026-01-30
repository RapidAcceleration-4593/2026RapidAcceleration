package frc.robot.util.mechanism;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.Dictionary;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public class Robot3D {
    private static Robot3D instance;

    private ArrayList<MechanismLigament3D> mechanisms = new ArrayList<>();
    private ArrayList<Runnable> updateRunnables = new ArrayList<>();

    public Robot3D() {
        AutoLogOutputManager.addObject(this);
    }

    public static Robot3D getInstance() {
        if (instance == null) {
            instance = new Robot3D();
        }
        return instance;
    }

    public void addMechanisms(Dictionary<MechanismLigament3D, Integer> mechsAndIndexes) {
        var iter = mechsAndIndexes.keys();
        while (iter.hasMoreElements()) {
            var mech = iter.nextElement();
            addMechanism(mech, mechsAndIndexes.get(mech));
        }
    }

    public void addMechanisms(MechanismLigament3D[] mechs) {
        for (int i = 0; i < mechs.length; i++) {
            addMechanism(mechs[i], i);
        }
    }

    public void addMechanism(MechanismLigament3D mech, int index) {
        var name = mech.getName();
        for (var iMech : mechanisms) {
            if (iMech == null) continue;
            if (name == iMech.getName() && !mech.equals(iMech)) {
                throw new Error("Mechanism with name '" + name + "' has already been added to this Robot3D!");
            }
        }

        while (mechanisms.size() <= index) {
            mechanisms.add(null);
        }

        if (mechanisms.get(index) != null) {
            throw new Error("Cannot put Mechanism '" + mech.getName() + "' at index " + index
                    + " because it is already occupied by '"
                    + mechanisms.get(index).getName() + "'!");
        }

        if (!mechanisms.contains(mech)) {
            mechanisms.set(index, mech);
        }

        var runnable = mech.getUpdateRunnable();
        if (updateRunnables.contains(runnable)) {
            updateRunnables.add(runnable);
        }
    }

    public MechanismLigament3D getMechanism(String name) {
        for (var mech : mechanisms) {
            if (name == mech.getName()) {
                return mech;
            }
        }
        throw new Error("Could not find Mechanism with name '" + name + "'!");
    }

    @AutoLogOutput
    private Pose3d[] log() {
        return (Pose3d[]) mechanisms.toArray();
    }
}
