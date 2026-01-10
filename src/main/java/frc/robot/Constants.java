package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static final class Controllers {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
    }
}
