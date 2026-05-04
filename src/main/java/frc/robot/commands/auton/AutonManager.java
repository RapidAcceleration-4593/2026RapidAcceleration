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
        autonMap.put("Custom", () -> new SideCenterCustom(util, true)); // TODO: Must be flipped manually.

        autonMap.put("LeftCenterTrench", () -> new SideCenterTrench(util, true));
        autonMap.put("LeftCenterBump", () -> new SideCenterBump(util, true));
        autonMap.put("LeftNoPickupNoTraversal", () -> new SideNoPickupNoTraversal(util, true));

        autonMap.put("CenterNoPickupNoTraversal", () -> new CenterNoPickupNoTraversal(util));

        autonMap.put("RightCenterTrench", () -> new SideCenterTrench(util, false));
        autonMap.put("RightCenterBump", () -> new SideCenterBump(util, false));
        autonMap.put("RightNoPickupNoTraversal", () -> new SideNoPickupNoTraversal(util, false));

        autonMap.put("Left2xCenterTrench", () -> new Side2xCenterTrench(util, true));
        autonMap.put("Right2xCenterTrench", () -> new Side2xCenterTrench(util, false));
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
            supplier = autonMap.get("DoNothing");
        }
        return supplier.get();
    }

    public Map<String, Supplier<Command>> getAutonMap() {
        return autonMap;
    }
}
