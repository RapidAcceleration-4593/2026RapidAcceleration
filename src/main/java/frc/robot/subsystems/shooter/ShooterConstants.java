package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public final class ShooterConstants {

    public static final SparkMax kShooterMotor = new SparkMax(0, MotorType.kBrushless);
    public static final double kShooterSpeed = 0.5;
}
