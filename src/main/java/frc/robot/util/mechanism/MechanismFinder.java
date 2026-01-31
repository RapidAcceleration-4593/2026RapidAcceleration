package frc.robot.util.mechanism;

public class MechanismFinder<T extends Mechanism3D> {
    private Class<T> clazz;

    public MechanismFinder(Class<T> clazz) {
        this.clazz = clazz;
    }

    public T find(String name) {
        return find(name, Robot3D.getInstance());
    }

    @SuppressWarnings("unchecked")
    public T find(String name, Robot3D source) {
        Mechanism3D mech = source.getMechanism(name);
        T casted;

        if (clazz.isInstance(mech)) {
            casted = (T) mech;
        } else {
            throw new Error("A Mechanism3D with name '" + name
                    + "' exists but is not an instance of the proper type. Expected type: " + clazz.getName()
                    + ". Actual type: " + mech.getClass().getName());
        }

        return casted;
    }

    public static final MechanismFinder<LengthMechanism3D> fLengthMechanism3D =
            new MechanismFinder<LengthMechanism3D>(LengthMechanism3D.class);
    public static final MechanismFinder<AngleMechanism3D> fAngleMechanism3D =
            new MechanismFinder<AngleMechanism3D>(AngleMechanism3D.class);
    public static final MechanismFinder<WheelMechanism3D> fWheelMechanism3D =
            new MechanismFinder<WheelMechanism3D>(WheelMechanism3D.class);
}
