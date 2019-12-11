package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.drivers.TalonSRXUtil;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

public class Flywheel extends Subsystem {
    private static Flywheel mInstance;

    public static Flywheel getInstance() {
        if (mInstance == null) {
            mInstance = new Flywheel();
        }

        return mInstance;
    }

    private final TalonSRX mMaster, mSlave;

    public class PeriodicIO {
        // inputs
        double velocity_ticks_per_100_ms = 0.0;
        
        // outputs
        double demand = 0.0;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private Flywheel() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kFlywheelMasterId);

        // initialize encoder
        TalonSRXUtil.checkError(mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
                Constants.kLongCANTimeoutMs), "Flywheel: Could not detect encoder: ");

        // set gains
        TalonSRXUtil.checkError(mMaster.config_kP(0, Constants.kFlywheelKp, Constants.kLongCANTimeoutMs),
            "Flywheel: could not set kP: ");
        TalonSRXUtil.checkError(mMaster.config_kI(0, Constants.kFlywheelKi, Constants.kLongCANTimeoutMs),
            "Flywheel: could not set kI: ");
        TalonSRXUtil.checkError(mMaster.config_kD(0, Constants.kFlywheelKd, Constants.kLongCANTimeoutMs),
            "Flywheel: could not set kD: ");
        TalonSRXUtil.checkError(mMaster.config_kF(0, Constants.kFlywheelKf, Constants.kLongCANTimeoutMs),
            "Flywheel: Could not set kF: ");

        mSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kFlywheelSlaveId, Constants.kFlywheelMasterId);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}
        
            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.velocity_ticks_per_100_ms = mMaster.getSelectedSensorPosition(0);
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.Velocity, mPeriodicIO.demand);
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public synchronized void setRPM(double rpm) {
        mPeriodicIO.demand = rpmToNativeUnits(rpm);
    }

    public synchronized double getRPM() {
        return nativeUnitsToRPM(getVelocityNativeUnits());
    }

    public synchronized double getVelocityNativeUnits() {
        return mPeriodicIO.velocity_ticks_per_100_ms;
    }

    /**
     * @param ticks per 100 ms
     * @return rpm
     */
    public double nativeUnitsToRPM(double ticks_per_100_ms) {
        return ticks_per_100_ms * 10.0 * 60.0 / Constants.kFlywheelTicksPerRevolution;
    }

    /**
     * @param rpm
     * @return ticks per 100 ms
     */
    public double rpmToNativeUnits(double rpm) {
        return rpm / 60.0 / 10.0 * Constants.kFlywheelTicksPerRevolution;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Flywheel RPM", getRPM());
    }
}