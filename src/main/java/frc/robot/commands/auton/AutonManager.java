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
        autonMap.put("DoNothing", () -> Commands.none());

        autonMap.put("LeftCenter", () -> new SideCenter(util, true));
        autonMap.put("LeftNoPickup", () -> new SideNoPickup(util, true));
        autonMap.put("Left2xCenter", () -> new Side2xCenter(util, true));
        autonMap.put("LeftCenterLoop", () -> new SideCenterLoop(util, true));

        autonMap.put("CenterNoPickup", () -> new CenterNoPickup(util));

        autonMap.put("RightCenter", () -> new SideCenter(util, false));
        autonMap.put("RightNoPickup", () -> new SideNoPickup(util, false));
        autonMap.put("Right2xCenter", () -> new Side2xCenter(util, false));
        autonMap.put("RightCenterLoop", () -> new SideCenterLoop(util, false));
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
