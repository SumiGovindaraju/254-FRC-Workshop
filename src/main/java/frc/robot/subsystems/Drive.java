package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;

public class Drive {
    private final TalonSRX mRightMaster;
    private final TalonSRX mRightSlave;
    private final TalonSRX mLeftMaster;
    private final TalonSRX mLeftSlave;

    public Drive() {
        mRightMaster = new TalonSRX(Constants.kDriveRightMasterId);
        mRightSlave = new TalonSRX(Constants.kDriveRightSlaveId);
        mRightSlave.set(ControlMode.Follower, Constants.kDriveRightMasterId);

        mLeftMaster = new TalonSRX(Constants.kDriveLeftMasterId);
        mLeftSlave = new TalonSRX(Constants.kDriveLeftSlaveId);
        mLeftSlave.set(ControlMode.Follower, Constants.kDriveLeftMasterId);
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

        mRightMaster.set(ControlMode.PercentOutput, right_demand);
        mLeftMaster.set(ControlMode.PercentOutput, left_demand);
    }

    public void stop() {
        mRightMaster.set(ControlMode.PercentOutput, 0.0);
        mLeftMaster.set(ControlMode.PercentOutput, 0.0);
    }
}