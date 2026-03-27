package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.center.*;
import frc.robot.commands.auton.side.*;
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
        autonMap.put("DoNothing", Commands::none);

        autonMap.put("LeftCenterTrench", () -> new SideCenter(util, true));
        autonMap.put("LeftCenterBump", () -> new SideCenterLoop(util, true));
        autonMap.put("LeftNoPickupNoTraversal", () -> new SideNoPickup(util, true));

        autonMap.put("CenterNoPickupNoTraversal", () -> new CenterNoPickup(util));

        autonMap.put("RightCenterTrench", () -> new SideCenter(util, false));
        autonMap.put("RightCenterBump", () -> new SideCenterLoop(util, false));
        autonMap.put("RightNoPickupNoTraversal", () -> new SideNoPickup(util, false));

        autonMap.put("Left2xCenterTrench", () -> new Side2xCenter(util, true));
        autonMap.put("Right2xCenterTrench", () -> new Side2xCenter(util, false));
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
