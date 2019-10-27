package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Drive extends Subsystem {
    private static Drive mInstance;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    public static class PeriodicIO {
        public double right_demand;
        public double left_demand;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonSRX mRightMaster;
    private final TalonSRX mRightSlave;
    private final TalonSRX mLeftMaster;
    private final TalonSRX mLeftSlave;

    private Drive() {
        mRightMaster = new TalonSRX(Constants.kDriveRightMasterId);
        mRightSlave = new TalonSRX(Constants.kDriveRightSlaveId);
        mRightSlave.set(ControlMode.Follower, Constants.kDriveRightMasterId);

        mLeftMaster = new TalonSRX(Constants.kDriveLeftMasterId);
        mLeftSlave = new TalonSRX(Constants.kDriveLeftSlaveId);
        mLeftSlave.set(ControlMode.Follower, Constants.kDriveLeftMasterId);
    }

    @Override
    public void writePeriodicOutputs() {
        mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
    }

    public void setOpenLoop(double throttle, double turn) {
        double right_demand = throttle + turn;
        double left_demand = throttle - turn;

        if (throttle == 0) {
            right_demand = 0.0;
            left_demand = 0.0;
        }

        if (Math.abs(right_demand) > 1.0 || Math.abs(left_demand) > 1.0) {
            right_demand /= Math.max(Math.abs(right_demand), Math.abs(left_demand));
            left_demand /= Math.max(Math.abs(right_demand), Math.abs(left_demand));
        }

        mPeriodicIO.right_demand = right_demand;
        mPeriodicIO.left_demand = left_demand;
    }

    @Override
    public void stop() {
        mRightMaster.set(ControlMode.PercentOutput, 0.0);
        mLeftMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {}
}