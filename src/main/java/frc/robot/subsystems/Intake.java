package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.TalonSRXFactory;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.statemachines.IntakeStateMachine;
import frc.robot.statemachines.IntakeStateMachine.IntakeState;
import frc.robot.statemachines.IntakeStateMachine.WantedState;

public class Intake extends Subsystem {
    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }

        return mInstance;
    }

    private final TalonSRX mLeftMaster, mRightMaster;
    private final Solenoid mCloseSolenoid, mClampSolenoid;

    private IntakeStateMachine mStateMachine = new IntakeStateMachine();

    private WantedState mWantedState = WantedState.IDLE;
    private IntakeState mCurrentState = new IntakeState();

    private Intake() {
        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kIntakeLeftTalonId);
        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kIntakeRightTalonId);

        mCloseSolenoid = new Solenoid(Constants.kPCMId, Constants.kIntakeCloseSolenoidId);
        mClampSolenoid = new Solenoid(Constants.kPCMId, Constants.kIntakeClampSolenoidId);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop(){
            @Override
            public void onStart(double timestamp) {
                mWantedState = WantedState.IDLE;
                mStateMachine = new IntakeStateMachine();
                mCurrentState = new IntakeState();
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

    public void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    @Override
    public void readPeriodicInputs() {
        mCurrentState.hasCube = false; // TODO: use an actual sensor value, but for now this is fine
    }

    @Override
    public void writePeriodicOutputs() {
        mLeftMaster.set(ControlMode.PercentOutput, mCurrentState.motor);
        mRightMaster.set(ControlMode.PercentOutput, mCurrentState.motor);
        mCloseSolenoid.set(mCurrentState.jawState.getCloseSolenoidValue());
        mClampSolenoid.set(mCurrentState.jawState.getClampSolenoidValue());
    }

    @Override
    public void stop() {
        mWantedState = WantedState.IDLE;
        mStateMachine = new IntakeStateMachine();
        mCurrentState = new IntakeState();

        mLeftMaster.set(ControlMode.PercentOutput, mCurrentState.motor);
        mRightMaster.set(ControlMode.PercentOutput, mCurrentState.motor);
        mCloseSolenoid.set(mCurrentState.jawState.getCloseSolenoidValue());
        mClampSolenoid.set(mCurrentState.jawState.getClampSolenoidValue());
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Has Cube", mCurrentState.hasCube);
    }
}