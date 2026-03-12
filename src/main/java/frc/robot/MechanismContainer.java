package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.util.mechanism.AngleMechanism3D;
import frc.robot.util.mechanism.Axis;
import frc.robot.util.mechanism.LengthMechanism3D;
import frc.robot.util.mechanism.Mechanism3D;
import frc.robot.util.mechanism.Node3D;
import frc.robot.util.mechanism.Robot3D;
import frc.robot.util.mechanism.WheelMechanism3D;

public class MechanismContainer {
    public MechanismContainer() {

        Node3D deployNode = new Node3D("Deploy", Pose3d.kZero);
        Node3D spindexerNode = new Node3D("Indexer", new Pose3d(0.034, 0, 0, Rotation3d.kZero));
        Node3D turretNode = new Node3D("Turret", new Pose3d(-0.144, 0, 0, Rotation3d.kZero));
        Node3D hoodNode = new Node3D("Hood", new Pose3d(0.12, 0, 0.47, new Rotation3d(0, 0, 0)), turretNode);
        hoodNode.setAngle(HoodConstants.kMinimumAngle, Axis.Y, false);
        Node3D climberNode = new Node3D("Climber", Pose3d.kZero);

        LengthMechanism3D deploy = new LengthMechanism3D(deployNode, Axis.X);
        WheelMechanism3D spindexer = new WheelMechanism3D(spindexerNode, Axis.Z);

        AngleMechanism3D turret = new AngleMechanism3D(turretNode, Axis.Z, true);
        AngleMechanism3D hood = new AngleMechanism3D(hoodNode, Axis.Y);
        LengthMechanism3D climber = new LengthMechanism3D(climberNode, Axis.Z);

        Robot3D.getInstance().addMechanisms(new Mechanism3D[] {deploy, spindexer, turret, hood, climber});
    }
}
