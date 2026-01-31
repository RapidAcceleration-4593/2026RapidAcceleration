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

    public Node3D(String name, Pose3d poseOffset) {
        this.name = name;
        this.poseOffset = poseOffset;
        this.children = new ArrayList<Node3D>();
    }

    public Node3D(String name, Pose3d poseOffset, Node3D parent) {
        this.name = name;
        this.poseOffset = poseOffset;
        this.children = new ArrayList<Node3D>();
        parent.addChild(this);
    }

    public void addChild(Node3D child) {
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
        child.parent = this;
        children.add(child);
    }

    public boolean hasParent() {
        return parent != null;
    }

    public String getName() {
        return this.name;
    }

    public Node3D[] getChildren() {
        return children.toArray(new Node3D[0]);
    }

    public Node3D getChildByName(String name) {
        for (var child : children) {
            if (child.name == name) {
                return child;
            }
        }
        return null;
    }

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

    public void setRelativePose(Pose3d pose) {
        relativePose = pose;
    }

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

    public Node3D getRootNode() {
        if (hasParent()) {
            return parent.getRootNode();
        }
        return this;
    }
}
