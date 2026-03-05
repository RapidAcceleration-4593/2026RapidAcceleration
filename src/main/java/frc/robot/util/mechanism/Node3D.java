package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;

/**
 * A class that simplifies 3D math by presenting a simple API for hierarchical 3D affine transforms. Each {@link Node3D}
 * may have a parent or children. The {@link Pose3d} of the parent affects the pose of the children. For instance,
 * moving the parent forward will also move the children forward. The rotational element of the {@link Pose3d} is
 * applied after the translational. The {@link Pose3d} represents the transformation in object space, not global space.
 */
public class Node3D {
    private Node3D parent;
    private List<Node3D> children;
    private Pose3d poseOffset;
    protected Pose3d relativePose = Pose3d.kZero;
    private Pose3d globalPose;
    private String name;
    private static final Alert addChildAlert = new Alert(
            "Failed to add a Mechanism3D as a child to another Mechanism3D. See console for more info.",
            AlertType.kWarning);

    /**
     * Creates a new {@link Node3D} with the given name and pose offset.
     *
     * @param name The name of the {@link Node3D}.
     * @param poseOffset This offset is added to the value passed in to {@link #setRelativePose()} to calculate the pose
     *     of this {@link Node3D} relative to its parent.
     */
    public Node3D(String name, Pose3d poseOffset) {
        this.name = name;
        this.poseOffset = poseOffset;
        this.children = new ArrayList<Node3D>();
    }

    /**
     * Creates a new {@link Node3D} with the given name, pose offset, and parent.
     *
     * @param name The name of the {@link Node3D}.
     * @param poseOffset This offset is added to the value passed in to {@link #setRelativePose()} to calculate the pose
     *     of this {@link Node3D} relative to its parent.
     * @param parent The {@link Node3D} that this {@link Node3D} inherits its position and rotation from.
     */
    public Node3D(String name, Pose3d poseOffset, Node3D parent) {
        this(name, poseOffset);
        parent.addChild(this);
    }

    /**
     * Adds a {@link Node3D} as a child to this {@link Node3D}. This is a no-op if the child is already a child of this
     * or another {@link Node3D}, or if this {@link Node3D} already has a child with the same name.
     *
     * @param child The {@link Node3D} to add as a child to this {@link Node3D}.
     */
    public void addChild(Node3D child) {
        if (child == null) {
            throw new IllegalArgumentException("Argument 'child' must not be null!");
        }
        if (children.contains(child)) {
            addChildAlert.set(true);
            System.err.println(
                    "Cannot add child. Node3D '" + child.name + "' is already a child of Node3D '" + name + "'!");
            return;
        }
        if (getChildByName(child.name) != null) {
            addChildAlert.set(true);
            System.err.println("Cannot add child. A different Node3D with name '" + child.name
                    + "' already exists as a child of Node3D '" + name + "'!");
            return;
        }
        if (child.parent != this && child.hasParent()) {
            addChildAlert.set(true);
            System.err.println("Cannot add '" + child.name + "' as a child to '" + name
                    + "'', as it is already the child of '" + child.parent + "'!'");
            return;
        }
        child.parent = this;
        children.add(child);
    }

    /**
     * Does this {@link Node3D} have a parent?
     *
     * @return True if this {@link Node3D} has a parent, false otherwise.
     */
    public boolean hasParent() {
        return parent != null;
    }

    /**
     * Returns the parent of this {@link Node3D}.
     *
     * @return This {@link Node3D}'s parent.
     */
    public Node3D getParent() {
        return parent;
    }

    /**
     * Returns the name of this {@link Node3D}.
     *
     * @return This {@link Node3D}'s name.
     */
    public String getName() {
        return this.name;
    }

    /**
     * Returns the children of this {@link Node3D}.
     *
     * @return An array of {@link Node3D}'s children.
     */
    public Node3D[] getChildren() {
        return children.toArray(new Node3D[0]);
    }

    /**
     * Returns the child {@link Node3D} that has the given name, null otherwise.
     *
     * @param name The name that the returned child should have.
     * @return The child that has the given name.
     */
    public Node3D getChildByName(String name) {
        for (var child : children) {
            if (child.name.equals(name)) {
                return child;
            }
        }
        return null;
    }

    /**
     * Returns the global pose (in the space of root node) as calculated by the last call to {@link #update()}.
     *
     * @return The global pose of this {@link Node3D}.
     */
    public Pose3d getGlobalPose() {
        return globalPose;
    }

    private Pose3d calculateZeroedPose() {
        Translation3d trans = relativePose.getTranslation().plus(poseOffset.getTranslation());
        Rotation3d relativeRot = relativePose.getRotation();
        Rotation3d offsetRot = poseOffset.getRotation();
        Rotation3d rot = new Rotation3d(
                relativeRot.getX() + offsetRot.getX(),
                relativeRot.getY() + offsetRot.getY(),
                relativeRot.getZ() + offsetRot.getZ());
        return new Pose3d(trans, rot);
    }

    /**
     * Recursively updates the position of this {@link Node3D} and all its children. For best performance, this should
     * only be called on the root {@link Node3D}.
     */
    public void update() {
        var adjustedPose = calculateZeroedPose();
        if (parent != null) {
            globalPose = new Pose3d(
                    parent.globalPose.getTranslation().plus(adjustedPose.getTranslation()), adjustedPose.getRotation());
            globalPose = globalPose.rotateAround(parent.globalPose.getTranslation(), parent.globalPose.getRotation());
        } else {
            globalPose = adjustedPose;
        }

        for (var child : children) {
            child.update();
        }
    }

    /**
     * Sets the relative {@link Pose3d} of this {@link Node3D}. This value is added to this {@link Node3D}'s pose
     * offset, and then transformed by this {@link Node3D}'s parent's global {@link Pose3d}. Note that the global pose
     * will not be updated until {@link #update()} is called on this.
     *
     * @param pose The relative {@link Pose3d} of this {@link Node3D}.
     */
    public void setRelativePose(Pose3d pose) {
        relativePose = pose;
    }

    /**
     * Sets the relative pose of this {@link Node3D} to a rotation by the given angle around the given axis, inverted if
     * necessary.
     *
     * @param angle The angle to rotate by.
     * @param axis The axis to rotate around.
     * @param inverted If true, the angle will be multiplied by -1 before further calculation.
     */
    public void setAngle(Angle angle, Axis axis, boolean inverted) {
        double angleRads = angle.in(Radians) * (inverted ? -1 : 1);
        switch (axis) {
            case X:
                setRelativePose(new Pose3d(0, 0, 0, new Rotation3d(angleRads, 0, 0)));
                break;
            case Y:
                setRelativePose(new Pose3d(0, 0, 0, new Rotation3d(0, angleRads, 0)));
                break;
            case Z:
                setRelativePose(new Pose3d(0, 0, 0, new Rotation3d(0, 0, angleRads)));
                break;
        }
    }

    /**
     * Sets the relative pose of this {@link Node3D} to a translation by the given distance on the given axis, inverted
     * if necessary.
     *
     * @param angle The distance to translate by.
     * @param axis The axis to translate on.
     * @param inverted If true, the distance will be multiplied by -1 before further calculation.
     */
    public void setRelativeDistance(Distance distance, Axis axis, boolean inverted) {
        double distanceMeters = distance.in(Meters) * (inverted ? -1 : 1);
        switch (axis) {
            case X:
                setRelativePose(new Pose3d(distanceMeters, 0, 0, Rotation3d.kZero));
                break;
            case Y:
                setRelativePose(new Pose3d(0, distanceMeters, 0, Rotation3d.kZero));
                break;
            case Z:
                setRelativePose(new Pose3d(0, 0, distanceMeters, Rotation3d.kZero));
                break;
        }
    }

    /**
     * Returns the nearest ancestor of this {@link Node3D} which has no parent. If this has no parent, it will return
     * this.
     *
     * @return The nearest ancestor {@link Node3D} that has no parent.
     */
    public Node3D getRootNode() {
        if (hasParent()) {
            return parent.getRootNode();
        }
        return this;
    }
}
