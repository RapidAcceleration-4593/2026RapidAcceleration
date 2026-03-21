package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.center.*;
import frc.robot.commands.auton.left.*;
import frc.robot.commands.auton.right.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Supplier;

public class AutonManager {

    private AutonUtil util;
    private final Map<String, Supplier<Command>> autonMap = new HashMap<>();

    public AutonManager(SwerveSubsystem swerve) {
        this.util = new AutonUtil(swerve);
        registerAutons();
    }

    private void registerAutons() {
        autonMap.put("LeftCenter", () -> new LeftCenter(util));
        autonMap.put("LeftNoPickup", () -> new LeftNoPickup(util));

        autonMap.put("CenterNoPickup", () -> new CenterNoPickup(util));

        autonMap.put("RightCenter", () -> new RightCenter(util));
        autonMap.put("RightNoPickup", () -> new RightNoPickup(util));
        autonMap.put("RightOutpost", () -> new RightOutpost(util));

        autonMap.put("DoNothing", () -> Commands.none());
        autonMap.put("Left2xCenter", () -> new Left2xCenter(util));
        autonMap.put("Right2xCenter", () -> new Right2xCenter(util));
        autonMap.put("RightCenterOutpost", () -> new RightCenterOutpost(util));

        autonMap.put("RightCenterLoop", () -> new RightCenterLoop(util));
    }

    /** Loads all PathPlanner paths into the cache. */
    public void warmup() {
        for (Entry<String, Supplier<Command>> entry : autonMap.entrySet()) {
            // Call the constructor of each auton to load the paths.
            entry.getValue().get();
        }
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
