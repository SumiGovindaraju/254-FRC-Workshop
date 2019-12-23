package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.TalonSRXFactory;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.statemachines.GearGrabberStateMachine;
import frc.robot.statemachines.GearGrabberStateMachine.GearGrabberState;
import frc.robot.statemachines.GearGrabberStateMachine.WantedState;

public class MotorGearGrabber extends Subsystem {
    private static MotorGearGrabber mInstance;

    public static MotorGearGrabber getInstance() {
        if (mInstance == null) {
            mInstance = new MotorGearGrabber();
        }

        return mInstance;
    }

    private GearGrabberStateMachine mStateMachine = new GearGrabberStateMachine();

    private final TalonSRX mMaster;
    private final Solenoid mSolenoid;

    private WantedState mWantedState = WantedState.IDLE;
    private GearGrabberState mCurrentState = new GearGrabberState();

    private MotorGearGrabber() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kMotorGearGrabberTalonId);
        mSolenoid = new Solenoid(Constants.kPCMId, Constants.kMotorGearGrabberSolenoidId);
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized GearGrabberState getGearGrabberState() {
        return mCurrentState;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop(){        
            @Override
            public void onStart(double timestamp) {
                mWantedState = WantedState.IDLE;
                mCurrentState = new GearGrabberState();
                mStateMachine = new GearGrabberStateMachine();
            }
        
            @Override
            public void onLoop(double timestamp) {
                mCurrentState = mStateMachine.update(timestamp, mWantedState, mCurrentState);
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        mCurrentState.hasGear = false; // TODO: use an actual sensor value, but for this demo it's fine
    }

    @Override
    public void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mCurrentState.motor);
        mSolenoid.set(mCurrentState.toggleState.getSolenoidValue());
    }

    @Override
    public void stop() {
        mWantedState = WantedState.IDLE;
        mCurrentState = new GearGrabberState();
        mStateMachine = new GearGrabberStateMachine();
        mMaster.set(ControlMode.PercentOutput, mCurrentState.motor);
        mSolenoid.set(mCurrentState.toggleState.getSolenoidValue());
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Has Gear", mCurrentState.hasGear);
    }
}