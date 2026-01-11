package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

public final class SwerveConstants {

    private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100)
            .withKI(0.0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(1.91)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs driveGains =
            new Slot0Configs().withKP(0.1).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.124);

    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    /** Current at which the wheels start to slip. */
    private static final Current kSlipCurrent = Amps.of(120.0);

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    // Help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true));

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    /** Configs for the Pigeon 2; leave null to skip applying Pigeon 2 configs. */
    private static final Pigeon2Configuration pigeonConfigs = null;

    /** CANBus/CANivore that all modules are connected to. */
    public static final CANBus kCANBus = new CANBus("drivebase", "./logs/example.hoot");

    /** Theoretical Free Speed at 12V. */
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.5);

    /** How much the drive motor unintentionally turns when you rotate steering (azimuth). */
    private static final double kCoupleRatio = 0.0; // Unknown

    private static final double kDriveGearRatio = 5.36;
    private static final double kSteerGearRatio = 18.75;
    private static final Distance kWheelRadius = Inches.of(2.0);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 5;

    /** Used only for simulation. */
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.05);

    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);

    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            ConstantCreator = new SwerveModuleConstantsFactory<
                            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(kDriveGearRatio)
                    .withSteerMotorGearRatio(kSteerGearRatio)
                    .withCouplingGearRatio(kCoupleRatio)
                    .withWheelRadius(kWheelRadius)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains)
                    .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                    .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                    .withSlipCurrent(kSlipCurrent)
                    .withSpeedAt12Volts(kSpeedAt12Volts)
                    .withDriveMotorType(kDriveMotorType)
                    .withSteerMotorType(kSteerMotorType)
                    .withFeedbackSource(kSteerFeedbackType)
                    .withDriveMotorInitialConfigs(driveInitialConfigs)
                    .withSteerMotorInitialConfigs(steerInitialConfigs)
                    .withEncoderInitialConfigs(encoderInitialConfigs)
                    .withSteerInertia(kSteerInertia)
                    .withDriveInertia(kDriveInertia)
                    .withSteerFrictionVoltage(kSteerFrictionVoltage)
                    .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left Module
    private static final int kFrontLeftDriveMotorId = 6;
    private static final int kFrontLeftSteerMotorId = 7;
    private static final int kFrontLeftEncoderId = 1;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.0);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(10.0);
    private static final Distance kFrontLeftYPos = Inches.of(10.0);

    // Front Right Module
    private static final int kFrontRightDriveMotorId = 8;
    private static final int kFrontRightSteerMotorId = 9;
    private static final int kFrontRightEncoderId = 2;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.0);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(10.0);
    private static final Distance kFrontRightYPos = Inches.of(-10.0);

    // Back Left Module
    private static final int kBackLeftDriveMotorId = 10;
    private static final int kBackLeftSteerMotorId = 11;
    private static final int kBackLeftEncoderId = 3;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.0);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-10.0);
    private static final Distance kBackLeftYPos = Inches.of(10.0);

    // Back Right Module
    private static final int kBackRightDriveMotorId = 12;
    private static final int kBackRightSteerMotorId = 13;
    private static final int kBackRightEncoderId = 4;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.0);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-10.0);
    private static final Distance kBackRightYPos = Inches.of(-10.0);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontLeft = ConstantCreator.createModuleConstants(
                    kFrontLeftSteerMotorId,
                    kFrontLeftDriveMotorId,
                    kFrontLeftEncoderId,
                    kFrontLeftEncoderOffset,
                    kFrontLeftXPos,
                    kFrontLeftYPos,
                    kInvertLeftSide,
                    kFrontLeftSteerMotorInverted,
                    kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            FrontRight = ConstantCreator.createModuleConstants(
                    kFrontRightSteerMotorId,
                    kFrontRightDriveMotorId,
                    kFrontRightEncoderId,
                    kFrontRightEncoderOffset,
                    kFrontRightXPos,
                    kFrontRightYPos,
                    kInvertRightSide,
                    kFrontRightSteerMotorInverted,
                    kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackLeft = ConstantCreator.createModuleConstants(
                    kBackLeftSteerMotorId,
                    kBackLeftDriveMotorId,
                    kBackLeftEncoderId,
                    kBackLeftEncoderOffset,
                    kBackLeftXPos,
                    kBackLeftYPos,
                    kInvertLeftSide,
                    kBackLeftSteerMotorInverted,
                    kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            BackRight = ConstantCreator.createModuleConstants(
                    kBackRightSteerMotorId,
                    kBackRightDriveMotorId,
                    kBackRightEncoderId,
                    kBackRightEncoderOffset,
                    kBackRightXPos,
                    kBackRightYPos,
                    kInvertRightSide,
                    kBackRightSteerMotorInverted,
                    kBackRightEncoderInverted);

    /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If
         * they need the devices, they can access them through getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If
         * they need the devices, they can access them through getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is
         *     250 Hz on CAN FD, and 100 Hz on CAN 2.0.
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, odometryUpdateFrequency, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If
         * they need the devices, they can access them through getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is
         *     250 Hz on CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation in the form [x, y, theta]ᵀ,
         *     with units in meters and radians
         * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y, theta]ᵀ, with
         *     units in meters and radians
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new,
                    TalonFX::new,
                    CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    odometryStandardDeviation,
                    visionStandardDeviation,
                    modules);
        }
    }
}
