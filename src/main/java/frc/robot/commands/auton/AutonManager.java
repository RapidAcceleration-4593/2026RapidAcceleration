package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auton.right.RightCenterOutpost;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class AutonManager {

    private AutonUtil util;
    private final Map<String, Supplier<Command>> autonMap = new HashMap<>();

    public AutonManager(SwerveSubsystem swerve) {
        this.util = new AutonUtil(swerve);
        registerAutons();
    }

    private void registerAutons() {
        autonMap.put("RightCenterOutpost", () -> new RightCenterOutpost(util));
    }

    public Command getAuton(String name) {
        Supplier<Command> supplier = autonMap.get(name);
        if (supplier == null) {
            System.err.println("Unknown autonomous routine: " + name + ". Defaulting.");
            supplier = autonMap.get("RightCenterOutpost");
        }
        return supplier.get();
    }

    public Map<String, Supplier<Command>> getAutonMap() {
        return autonMap;
    }
}
