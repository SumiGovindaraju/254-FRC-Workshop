package frc.robot;

public class Constants {
    public static final double kLooperDt = 0.01;

    // CAN
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates

    // Pneumatics
    public static final int kPCMId = 0;

    // Drive
    public static final int kDriveRightMasterId = 0;
    public static final int kDriveRightSlaveId = 1;
    public static final int kDriveLeftMasterId = 2;
    public static final int kDriveLeftSlaveId = 3;

    public static final double kDriveWheelTrackWidthInches = 25.42;
    public static final double kTrackScrubFactor = 1.0469745223;

    // Joysticks
    public static final int kThrottleStickPort = 0;
    public static final int kTurnStickPort = 1;


    // Flywheel
    public static final int kFlywheelMasterId = 4;
    public static final int kFlywheelSlaveId = 5;
    public static final double kFlywheelKp = 0.0;
    public static final double kFlywheelKi = 0.0;
    public static final double kFlywheelKd = 0.0;
    public static final double kFlywheelKf = 0.0;
    public static final double kFlywheelTicksPerRevolution = 0.0; // based on gear reduction between encoder and output shaft, and encoder ppr

    // Gear Grabber
    public static final int kMotorGearGrabberTalonId = 6;
    public static final int kMotorGearGrabberSolenoidId = 1;

    // Intake
    public static final int kIntakeLeftTalonId = 7;
    public static final int kIntakeRightTalonId = 8;
    public static final int kIntakeCloseSolenoidId = 2;
    public static final int kIntakeClampSolenoidId = 3;
}