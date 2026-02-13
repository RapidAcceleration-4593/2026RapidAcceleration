package frc.robot.util.tuning;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class SimPoseTuner {
    private static String tableName = "Tuning";
    private String fullName;

    private LoggedNetworkNumber xPos, yPos, zPos, xRot, yRot, zRot;
    private String name;

    public SimPoseTuner(String name, Pose3d pose) {
        this.name = name;
        this.fullName = tableName + "/" + name;

        xPos = new LoggedNetworkNumber(fullName + "/x");
        yPos = new LoggedNetworkNumber(fullName + "/y");
        zPos = new LoggedNetworkNumber(fullName + "/z");
        xRot = new LoggedNetworkNumber(fullName + "/xRot");
        yRot = new LoggedNetworkNumber(fullName + "/yRot");
        zRot = new LoggedNetworkNumber(fullName + "/zRot");
        xPos.set(pose.getX());
        yPos.set(pose.getY());
        zPos.set(pose.getZ());
        xRot.set(pose.getRotation().getX());
        yRot.set(pose.getRotation().getY());
        zRot.set(pose.getRotation().getZ());
    }

    @AutoLogOutput(key = "{tablename}/{name}")
    public Pose3d getPose() {
        return new Pose3d(xPos.get(), yPos.get(), zPos.get(), new Rotation3d(xRot.get(), yRot.get(), zRot.get()));
    }

    public String getName() {
        return name;
    }
}
