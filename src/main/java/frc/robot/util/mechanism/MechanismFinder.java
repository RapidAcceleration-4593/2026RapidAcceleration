package frc.robot.util.mechanism;

/**
 * Instances of this class offer a compact, type safe way of accessing {@link Mechanism3D} subclasses of a given type.
 */
public final class MechanismFinder<T extends Mechanism3D> {
    private Class<T> clazz;

    /**
     * Creates a new {@link MechanismFinder} for a given type of {@link Mechanism3D}. In order to have proper type
     * checking at runtime the class of the type that this {@link Mechanism3D} is purposed for must be passed in.
     *
     * @param clazz
     */
    public MechanismFinder(Class<T> clazz) {
        this.clazz = clazz;
    }

    /**
     * Searches for a {@link Mechanism3D} with the given name of type T in the singleton instance of {@link Robot3D}.
     *
     * @param name The name that the {@link Mechanism3D} should have.
     * @return A {@link Mechanism3D} with the matching name.
     */
    public T find(String name) {
        return find(name, Robot3D.getInstance());
    }

    /**
     * Searches for a {@link Mechanism3D} with the given name of type T in the given {@link Robot3D}.
     *
     * @param name The name that the {@link Mechanism3D} should have.
     * @param source The {@link Robot3D} that this will search through.
     * @return A {@link Mechanism3D} with the matching name.
     */
    public T find(String name, Robot3D source) {
        Mechanism3D mech = source.getMechanism(name);
        try {
            return clazz.cast(mech);
        } catch (ClassCastException e) {
            throw new IllegalArgumentException("A Mechanism3D with name '" + name
                    + "' exists but is not an instance of the proper type. Expected type: " + clazz.getName()
                    + ". Actual type: " + mech.getClass().getName());
        }
    }

    public static final MechanismFinder<LengthMechanism3D> fLengthMechanism3D =
            new MechanismFinder<LengthMechanism3D>(LengthMechanism3D.class);
    public static final MechanismFinder<AngleMechanism3D> fAngleMechanism3D =
            new MechanismFinder<AngleMechanism3D>(AngleMechanism3D.class);
    public static final MechanismFinder<WheelMechanism3D> fWheelMechanism3D =
            new MechanismFinder<WheelMechanism3D>(WheelMechanism3D.class);
}
