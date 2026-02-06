package frc.robot.util.mechanism;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.Dictionary;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

public class Robot3D {
    private static Robot3D instance;

    private ArrayList<Mechanism3D> mechanisms = new ArrayList<>();
    private ArrayList<Runnable> updateRunnables = new ArrayList<>();

    /**
     * Creates a new {@link Robot3D}. The returned instance is not set as the singleton. Access to the singleton should
     * always be done through {@link #getInstance()}.
     */
    public Robot3D() {
        AutoLogOutputManager.addObject(this);
    }

    /**
     * Returns the singleton instance of the {@link Robot3D}.
     *
     * @return The singleton.
     */
    public static Robot3D getInstance() {
        if (instance == null) {
            instance = new Robot3D();
        }
        return instance;
    }

    /**
     * Adds a dictionary of {@link Mechanism3D}s to this {@link Robot3D}.
     *
     * @param mechsAndIndexes The dictionary with a {@link Mechanism3D} as the key and the index that it should be
     *     logged at as the value.
     */
    public void addMechanisms(Dictionary<Mechanism3D, Integer> mechsAndIndexes) {
        var iter = mechsAndIndexes.keys();
        while (iter.hasMoreElements()) {
            var mech = iter.nextElement();
            addMechanism(mech, mechsAndIndexes.get(mech));
        }
    }

    /**
     * Appends a list of {@link Mechanism3D}s to the current list of {@link Mechanism3D}s.
     *
     * @param mechs The list of {@link Mechanism3D}s.
     */
    public void addMechanisms(Mechanism3D[] mechs) {
        for (int i = 0; i < mechs.length; i++) {
            addMechanism(mechs[i], i);
        }
    }

    /**
     * Adds a {@link Mechanism3D} at the given index.
     *
     * @param mech The {@link Mechanism3D} to add.
     * @param index The index where the {@link Mechanism3D} should be placed.
     */
    public void addMechanism(Mechanism3D mech, int index) {
        var name = mech.getName();
        for (var iMech : mechanisms) {
            if (iMech == null) continue;
            if (name.equals(iMech.getName()) && !mech.equals(iMech)) {
                throw new IllegalArgumentException(
                        "Mechanism with name '" + name + "' has already been added to this Robot3D!");
            }
        }

        while (mechanisms.size() <= index) {
            mechanisms.add(null);
        }

        if (mechanisms.get(index) != null) {
            throw new IllegalArgumentException("Cannot put Mechanism '" + mech.getName() + "' at index " + index
                    + " because it is already occupied by '"
                    + mechanisms.get(index).getName() + "'!");
        }

        if (!mechanisms.contains(mech)) {
            mechanisms.set(index, mech);
        }

        var runnable = mech.getUpdateRunnable();
        if (!updateRunnables.contains(runnable)) {
            updateRunnables.add(runnable);
        }
    }

    /**
     * Searches for and returns a {@link Mechanism3D} with a matching name. Throws an error if no match is found.
     *
     * @param name The name the returned {@link Mechanism3D} should have.
     * @return A {@link Mechanism3D} with a matching name.
     */
    Mechanism3D getMechanism(String name) {
        for (var mech : mechanisms) {
            if (mech == null) {
                continue;
            }
            if (name.equals(mech.getName())) {
                return mech;
            }
        }
        throw new IllegalArgumentException("Could not find Mechanism with name '" + name + "'!");
    }

    @AutoLogOutput(key = "MechanismPoses")
    private Pose3d[] log() {
        for (var updater : updateRunnables) {
            updater.run();
        }
        Pose3d[] poses = new Pose3d[mechanisms.size()];
        for (int i = 0; i < mechanisms.size(); i++) {
            poses[i] = mechanisms.get(i).getPose3d();
        }
        return poses;
    }
}
