package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.center.*;
import frc.robot.commands.auton.left.*;
import frc.robot.commands.auton.right.*;
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
        autonMap.put("LeftCenterNoClimb", () -> new LeftCenterNoClimb(util));
        autonMap.put("LeftNoPickupNoClimb", () -> new LeftNoPickupNoClimb(util));

        autonMap.put("CenterNoPickupNoClimb", () -> new CenterNoPickupNoClimb(util));

        autonMap.put("RightCenterNoClimb", () -> new RightCenterNoClimb(util));
        autonMap.put("RightNoPickupNoClimb", () -> new RightNoPickupNoClimb(util));
        autonMap.put("RightOutpostNoClimb", () -> new RightOutpostNoClimb(util));

        autonMap.put("DoNothing", () -> Commands.none());
        autonMap.put("Left2xCenterNoClimb", () -> new Left2xCenterNoClimb(util));
        autonMap.put("Right2xCenterNoClimb", () -> new Right2xCenterNoClimb(util));
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
