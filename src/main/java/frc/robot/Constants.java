package frc.robot;

public class Constants {
    public static final double kLooperDt = 0.01;

    // CAN
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates

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
}