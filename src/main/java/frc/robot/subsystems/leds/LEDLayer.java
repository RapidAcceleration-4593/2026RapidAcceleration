package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDLayer {

    public int patternIndex = 0;
    public Color baseColor = Color.kBlue;
    public Color gradientColor = Color.kBlack;
    public boolean useAllianceColor = false;
    public double speedFactor = 1.0;

    public double priority = 0.0;
}
