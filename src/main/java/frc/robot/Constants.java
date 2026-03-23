package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

    public static final class Controllers {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }
}
